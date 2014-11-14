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

#include "qhull_calculator.h"
#include "rtt/Component.hpp"

QhullCalculator::QhullCalculator(const std::string& name) :
	RTT::TaskContext(name)
{
	this->ports()->addPort("QhullDataOut", qhull_data_out_);
	this->ports()->addPort("QhullPointsIn", qhull_points_in_);
}

QhullCalculator::~QhullCalculator() {
}

bool QhullCalculator::configureHook() {
	char qhull_command_const[] = "Qt i";
	memcpy(qhull_command_, qhull_command_const, strlen(qhull_command_const) + 1);

	face_i_ = new int[QhullData::MAX_POINTS];

	//
	// initialize qhull
	//
//	static FILE *null_sink = fopen("/dev/null", "w");
//	qh_init_A(NULL, null_sink, null_sink, 0, NULL);
	qh_init_A(NULL, stdout, stdout, 0, NULL);
	qh_initflags( qhull_command_ );
}

void QhullCalculator::cleanupHook() {
	delete[] face_i_;
	qh_freeqhull( True);
}

bool QhullCalculator::startHook() {
	return true;
}

void QhullCalculator::updateHook() {
	qhull_points_in_.read(qhull_points_);


	qhull_data_.num_points = 0;
	qhull_data_.num_planes = 0;
	qhull_data_.error = false;

	if (qhull_points_.num_points < 4)
	{
		qhull_data_.error = true;
		return;
	}



	//
	// run qhull
	//

	// points_ are going to be free'd in qh_freebuffers
	points_ = new coordT[3 * QhullData::MAX_POINTS];

	// initialize points
	for (int i=0; i<qhull_points_.num_points; i++)
	{
		points_[i*3+0] = qhull_points_.points[i].x();
		points_[i*3+1] = qhull_points_.points[i].y();
		points_[i*3+2] = qhull_points_.points[i].z();
	}

	qh_init_B(points_, qhull_points_.num_points, 3, True);
	qh_qhull();

	std::map<int, int> v_id_map;
	for (vertexT *v = qh vertex_list; v != qh vertex_tail; v = v->next)
	{
		v_id_map[v->id] = qhull_data_.num_points;
		qhull_data_.points[qhull_data_.num_points] = KDL::Vector(v->point[0], v->point[1], v->point[2]);
		qhull_data_.num_points++;
	}

	int polygons_idx = 0;
	for (facetT *f = qh facet_list; f != qh facet_tail; f = f->next)
	{
		for (int v_idx=0; v_idx < f->vertices->maxsize; v_idx++)
		{
			if (f->vertices->e[v_idx].p == NULL)
				break;
			vertexT *v = static_cast<vertexT *>(f->vertices->e[v_idx].p);

			if (v_idx >= QhullData::MAX_POINTS)
			{
				std::cout << "v_idx >= QhullData::MAX_POINTS" << std::endl;
				qhull_data_.error = true;
				break;
			}
			if (v_id_map.find(v->id) == v_id_map.end())
			{
				std::cout << "v_id in polygon is not in vertex list" << std::endl;
				qhull_data_.error = true;
				break;
			}
			face_i_[v_idx] = v_id_map[v->id];
			face_count_ = v_idx+1;
		}

		if (face_count_ < 3)
		{
			std::cout << "degenerated facet: " << face_count_ << std::endl;
			qhull_data_.error = true;
			continue;
		}
		if (qhull_data_.error)
		{
			break;
		}

		// triangle facet is reliable
		if (face_count_ == 3)
		{
			if (polygons_idx+3 >= QhullData::MAX_POLYGON_DATA_LENGTH)
			{
				qhull_data_.error = true;
				break;
			}
			qhull_data_.polygons[polygons_idx++] = face_count_;
			qhull_data_.polygons[polygons_idx++] = face_i_[0];
			qhull_data_.polygons[polygons_idx++] = face_i_[1];
			qhull_data_.polygons[polygons_idx++] = face_i_[2];
			qhull_data_.num_planes++;
			continue;
		}

		// the facet has more than 3 vertices - we must get edges to construct proper polygon
		face_count_ = 0;

		std::multimap<int, int> ridges_map;
		// get the ridges
		if (f->ridges == NULL)
		{
			qhull_data_.error = true;
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
						qhull_data_.error = true;
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
			if (v_idx >= QhullData::MAX_POINTS)
			{
				qhull_data_.error = true;
				std::cout << "too many vertices in one polygon" << std::endl;
				break;
			}
		}

		qhull_data_.polygons[polygons_idx++] = face_count_;
		for (int face_i_idx=0; face_i_idx<face_count_; face_i_idx++)
		{
			qhull_data_.polygons[polygons_idx++] = face_i_[face_i_idx];
		}
		qhull_data_.num_planes++;
	}

	//
	// free qhull
	//
	qh_freebuffers();

	int curlong, totlong;
	qh_memfreeshort(&curlong, &totlong);
	if (curlong || totlong)
		std::cout << "qhull internal warning (main): did not free " << totlong << " bytes of long memory(" << curlong << " pieces)\n" << std::endl;


	qhull_data_out_.write(qhull_data_);
}

ORO_CREATE_COMPONENT(QhullCalculator)

