/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Dawid Seredynski */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_kdl/tf2_kdl/tf2_kdl.h>
#include <sstream>
#include "qhull_msgs/QhullList.h"
#include "qhull_msgs/PointLists.h"

extern "C" {
#include <libqhull/qset.h>
#include <libqhull/libqhull.h>
}

void caclulateQhull(int num_points_in, const std::vector<geometry_msgs::Point> &points_in, int &num_points, std::vector<geometry_msgs::Point> &points, int &num_planes, std::vector<int> &polygons)
{
	num_points = 0;
	num_planes = 0;

	if (num_points_in < 4)
	{
		return;
	}

	// TODO: vectors' range checks, check if there is memory leak and test for stability...

	//
	// initialize qhull
	//
	char qhull_command_const[] = "Qt i";
	char qhull_command_[10];
	memcpy(qhull_command_, qhull_command_const, strlen(qhull_command_const) + 1);

//	static FILE *null_sink = fopen("/dev/null", "w");
//	qh_init_A(NULL, null_sink, null_sink, 0, NULL);
	qh_init_A(NULL, stdout, stdout, 0, NULL);
	qh_initflags( qhull_command_ );

	//
	// run qhull
	//

	// points_ are going to be free'd in qh_freebuffers
	coordT *points_ = new coordT[3 * num_points_in];
	int *face_i_ = new int[num_points_in];
	int face_count_ = 0;

	// initialize points
	for (int i=0; i<num_points_in; i++)
	{
		points_[i*3+0] = points_in[i].x;
		points_[i*3+1] = points_in[i].y;
		points_[i*3+2] = points_in[i].z;
	}

	qh_init_B(points_, num_points_in, 3, True);
	qh_qhull();

	std::map<int, int> v_id_map;
	for (vertexT *v = qh vertex_list; v != qh vertex_tail; v = v->next)
	{
		v_id_map[v->id] = num_points;
		geometry_msgs::Point pt;
		pt.x = v->point[0];
		pt.y = v->point[1];
		pt.z = v->point[2];
		points.push_back(pt);
		num_points++;
	}

	int polygons_idx = 0;
	for (facetT *f = qh facet_list; f != qh facet_tail; f = f->next)
	{
		for (int v_idx=0; v_idx < f->vertices->maxsize; v_idx++)
		{
			if (f->vertices->e[v_idx].p == NULL)
				break;
			vertexT *v = static_cast<vertexT *>(f->vertices->e[v_idx].p);

//			if (v_idx >= QhullData::MAX_POINTS)
//			{
//				std::cout << "v_idx >= QhullData::MAX_POINTS" << std::endl;
//				break;
//			}
			if (v_id_map.find(v->id) == v_id_map.end())
			{
				std::cout << "v_id in polygon is not in vertex list" << std::endl;
				break;
			}
			face_i_[v_idx] = v_id_map[v->id];
			face_count_ = v_idx+1;
		}

		if (face_count_ < 3)
		{
			std::cout << "degenerated facet: " << face_count_ << std::endl;
			continue;
		}

		// triangle facet is reliable
		if (face_count_ == 3)
		{
//			if (polygons_idx+3 >= QhullData::MAX_POLYGON_DATA_LENGTH)
//			{
//				break;
//			}
			polygons.push_back(face_count_);
			polygons.push_back(face_i_[0]);
			polygons.push_back(face_i_[1]);
			polygons.push_back(face_i_[2]);
			num_planes++;
			continue;
		}

		// the facet has more than 3 vertices - we must get edges to construct proper polygon
		face_count_ = 0;

		std::multimap<int, int> ridges_map;
		// get the ridges
		if (f->ridges == NULL)
		{
			break;
		}
		else
		{
			for (int r_idx=0; r_idx < f->ridges->maxsize; r_idx++)
			{
				if (f->ridges->e[r_idx].p == NULL)
					break;
				ridgeT *r = static_cast<ridgeT *>(f->ridges->e[r_idx].p);
				if (r->vertices == NULL)
				{
//					std::cout << "NULL" << std::endl;
				}
				else
				{
					int ridge[2] = {-1, -1};
					for (int v_idx=0; v_idx < r->vertices->maxsize; v_idx++)
					{
						if (r->vertices->e[v_idx].p == NULL)
							break;
						vertexT *v = static_cast<vertexT *>(r->vertices->e[v_idx].p);
						if (v_idx > 1)
						{
							std::cout << "ridge has over 2 vertices" << std::endl;
							break;
						}
						ridge[v_idx] = v->id;
					}
					if (ridge[0] == -1 || ridge[1] == -1)
					{
						std::cout << "wrong ridge: " << ridge[0] << " " << ridge[1] << std::endl;
						break;
					}
					else
					{
						ridges_map.insert(std::make_pair<int,int>(ridge[0], ridge[1]));
						ridges_map.insert(std::make_pair<int,int>(ridge[1], ridge[0]));
					}
				}
			}
		}

		int init_v_id = ridges_map.begin()->first;
		int v_id_prev = -1;
		int v_id = init_v_id;
		int v_idx = 0;

		while (true)
		{
			face_i_[v_idx] = v_id_map[v_id];
			face_count_ = v_idx+1;
			v_idx++;
			std::pair<std::multimap<int, int>::iterator, std::multimap<int, int>::iterator> range = ridges_map.equal_range(v_id);
			for (std::multimap<int, int>::iterator it = range.first; it != range.second; it++)
			{
				if (it->second != v_id_prev)
				{
					v_id_prev = v_id;
					v_id = it->second;
					break;
				}
			}
			if (v_id == init_v_id)
				break;
//			if (v_idx >= QhullData::MAX_POINTS)
//			{
//				std::cout << "too many vertices in one polygon" << std::endl;
//				break;
//			}
		}

		polygons.push_back(face_count_);
		for (int face_i_idx=0; face_i_idx<face_count_; face_i_idx++)
		{
			polygons.push_back(face_i_[face_i_idx]);
		}
		num_planes++;
	}

//	std::cout << "freeing qhull..." << std::endl;
	//
	// free qhull
	//
	qh_freebuffers();
	qh_freeqhull( True);

//	int curlong, totlong;
//	qh_memfreeshort(&curlong, &totlong);
//	if (curlong || totlong)
//		std::cout << "qhull internal warning (main): did not free " << totlong << " bytes of long memory(" << curlong << " pieces)\n" << std::endl;

	delete[] face_i_;
}

ros::Publisher pub;

void qhullCallback(const qhull_msgs::PointLists& msg)
{
//	ROS_INFO("msg.point_lists.size() == %ld", msg.point_lists.size());
	qhull_msgs::QhullList qhulls;
	qhulls.qhulls.resize(msg.point_lists.size());
	for (int i=0; i<msg.point_lists.size(); i++)
	{
//		ROS_INFO("caclulateQhull for %d: %d", i, msg.point_lists[i].num_points);
		caclulateQhull(msg.point_lists[i].num_points, msg.point_lists[i].points, qhulls.qhulls[i].num_points, qhulls.qhulls[i].points, qhulls.qhulls[i].num_planes, qhulls.qhulls[i].polygons);
	}
	pub.publish(qhulls);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qhull_calculator");
	ros::NodeHandle n;

	pub = n.advertise<qhull_msgs::QhullList>("/qhull", 10);

	ros::Subscriber sub = n.subscribe("/qhull_points", 10, qhullCallback);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

