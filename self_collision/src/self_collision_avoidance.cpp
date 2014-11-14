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

#include "self_collision_avoidance.h"
#include "rtt/Component.hpp"

#include "urdf/model.h"
#include <kdl_parser/kdl_parser.hpp>

void Distance::addMarkers(visualization_msgs::MarkerArray &marker_array)
{
	marker_id_ = 0;
	if (marker_array.markers.size() > 0)
	{
		marker_id_ = marker_array.markers.back().id + 1;
	}

	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.ns = "default";
	marker.id = marker_id_;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.points.resize(2);
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;
	marker.color.a = 1.0;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 0;
	marker_array.markers.push_back(marker);

	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "world";
	marker2.ns = "default";
	marker2.id = marker_id_+1;
	marker2.type = visualization_msgs::Marker::SPHERE;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.pose.orientation.x = 0.0;
	marker2.pose.orientation.y = 0.0;
	marker2.pose.orientation.z = 0.0;
	marker2.pose.orientation.w = 1.0;
	marker2.scale.x = 0.02;
	marker2.scale.y = 0.02;
	marker2.scale.z = 0.02;
	marker2.color.a = 1.0;
	marker2.color.r = 1;
	marker2.color.g = 0;
	marker2.color.b = 0;
	marker_array.markers.push_back(marker2);

	visualization_msgs::Marker marker3;
	marker3.header.frame_id = "world";
	marker3.ns = "default";
	marker3.id = marker_id_+2;
	marker3.type = visualization_msgs::Marker::SPHERE;
	marker3.action = visualization_msgs::Marker::ADD;
	marker3.pose.orientation.x = 0.0;
	marker3.pose.orientation.y = 0.0;
	marker3.pose.orientation.z = 0.0;
	marker3.pose.orientation.w = 1.0;
	marker3.scale.x = 0.02;
	marker3.scale.y = 0.02;
	marker3.scale.z = 0.02;
	marker3.color.a = 1.0;
	marker3.color.r = 0;
	marker3.color.g = 1;
	marker3.color.b = 0;
	marker_array.markers.push_back(marker3);
}

void Distance::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &T_B_i)
{
	KDL::Vector pos1 = T_B_i * xi_;
	KDL::Vector pos2 = T_B_i * xj_;

	if (marker_id_+2 >= marker_array.markers.size())
	{
		std::cout<<"Distance::updateMarkers ERROR!"<<std::endl;
		*((int*)NULL) = 1;
	}

	marker_array.markers[marker_id_].header.stamp = ros::Time();
	marker_array.markers[marker_id_].points.at(0).x = pos1.x();
	marker_array.markers[marker_id_].points.at(0).y = pos1.y();
	marker_array.markers[marker_id_].points.at(0).z = pos1.z();
	marker_array.markers[marker_id_].points.at(1).x = pos2.x();
	marker_array.markers[marker_id_].points.at(1).y = pos2.y();
	marker_array.markers[marker_id_].points.at(1).z = pos2.z();

	marker_array.markers[marker_id_+1].header.stamp = ros::Time();
	marker_array.markers[marker_id_+1].pose.position.x = pos1.x();
	marker_array.markers[marker_id_+1].pose.position.y = pos1.y();
	marker_array.markers[marker_id_+1].pose.position.z = pos1.z();

	marker_array.markers[marker_id_+2].header.stamp = ros::Time();
	marker_array.markers[marker_id_+2].pose.position.x = pos2.x();
	marker_array.markers[marker_id_+2].pose.position.y = pos2.y();
	marker_array.markers[marker_id_+2].pose.position.z = pos2.z();
}

SelfCollisionAvoidance::SelfCollisionAvoidance(const std::string& name) :
	RTT::TaskContext(name),
	joints_count_(0)
{
	this->addProperty("robot_description", prop_robot_description_);
	this->addProperty("robot_semantic_description", prop_robot_semantic_description_);
	this->addProperty("distances_count", prop_distances_count_);
	this->addProperty("d0", prop_d0_);
	this->ports()->addPort("dbg_joint_states", joint_in_);
	this->ports()->addPort("dbg_markers", markers_out_);

	this->ports()->addPort("QhullDataIn", qhull_data_in_);
	this->ports()->addPort("QhullPointsOut", qhull_points_out_);
}

SelfCollisionAvoidance::~SelfCollisionAvoidance() {
}

bool SelfCollisionAvoidance::configureHook() {

	// read KDL::Tree form the robot_description
	if (!kdl_parser::treeFromString(prop_robot_description_, robot_tree_))
	{
		RTT::log(RTT::Error) << "could not read robot_description parameter" << std::endl;
		return false;
	}

	// read the robot model from the robot_description
	urdf::Model robot_model_;
	robot_model_.initString(prop_robot_description_);

	collision_model_ = self_collision::CollisionModel::parseURDF(prop_robot_description_);
	collision_model_->parseSRDF(prop_robot_semantic_description_);
	collision_model_->generateCollisionPairs();

	// update the links graph with KDL data
	for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
	{
		collision_model_->links_[l_i]->kdl_segment_ = &(robot_tree_.getSegment(collision_model_->links_[l_i]->name)->second);
	}

	// prepare the map of joint states
	for (KDL::SegmentMap::const_iterator seg_it = robot_tree_.getSegments().begin(); seg_it != robot_tree_.getSegments().end(); seg_it++)
	{
		KDL::Joint::JointType type = seg_it->second.segment.getJoint().getType();
		if (	type == KDL::Joint::RotAxis ||
			type == KDL::Joint::RotX ||
			type == KDL::Joint::RotY ||
			type == KDL::Joint::RotZ)
		{
			std::string j_name = seg_it->second.segment.getJoint().getName();
			std::string l_name = seg_it->second.segment.getName();
			int l_index = -1;
			for (int l_i=0; l_i<collision_model_->link_count_; l_i++)
			{
				if (l_name == collision_model_->links_[l_i]->name)
				{
					l_index = l_i;
					break;
				}
			}
			if (l_index == -1)
			{
				RTT::log(RTT::Error) << "could not find link: " << l_name << std::endl;
				return false;
			}
			RTT::log(RTT::Info) << "joint_name_2_index_map_: " << j_name << " " << l_name << " " << l_index << std::endl;
			joint_name_2_index_map_.insert(std::make_pair<std::string,int>(j_name,l_index));
		}		
	}

	joints_count_ = joint_name_2_index_map_.size();
	RTT::log(RTT::Info) << "number of joints:" << joints_count_ << std::endl;

	joint_positions_by_index_.resize(collision_model_->link_count_);

	transformations_by_index_.resize(collision_model_->link_count_);

	//
	// create vector of proper fk calculation link sequence (sort the links graph)
	//
	fk_seq_.resize(collision_model_->link_count_);
	// update parent link information
	for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
	{
		urdf::Link *parent_link = robot_model_.getLink( collision_model_->links_[l_i]->name )->getParent().get();
		if (parent_link != NULL)
		{
			RTT::log(RTT::Info) << "link: " << collision_model_->links_[l_i]->name << ", parent: " << collision_model_->links_[collision_model_->getLinkIndex( parent_link->name )]->name << std::endl;
			collision_model_->links_[l_i]->parent_index_ = collision_model_->getLinkIndex( parent_link->name );
		}
		else
		{
			RTT::log(RTT::Info) << "link: " << collision_model_->links_[l_i]->name << ", parent: NULL" << std::endl;
			collision_model_->links_[l_i]->parent_index_ = -1;
			fk_seq_[0] = l_i;
		}
	}
	// sort the graph
	int fk_seq_i = 1;
	while (true)
	{
		bool added = false;
		int fk_seq_i_saved = fk_seq_i;
		for (int i=0; i<fk_seq_i_saved; i++)
		{
			for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
			{
				// found child of already added link
				if (collision_model_->links_[l_i]->parent_index_ == fk_seq_[i])
				{
					// check if the child was added
					bool child_already_added = false;
					for (int j=0; j<fk_seq_i; j++)
					{
						if (fk_seq_[j] == l_i)
						{
							child_already_added = true;
							break;
						}
					}
					// add child
					if (!child_already_added)
					{
						added = true;
						fk_seq_[fk_seq_i] = l_i;
						fk_seq_i++;
					}
				}
			}
		}
		if (!added)
		{
			break;
		}
	}

	// create vector of distances
	distances.resize(prop_distances_count_);

	// create vector of convex hulls for quick update
	// iterate through all links
	for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
	{
		// iterate through collision objects
		for (self_collision::Link::VecPtrCollision::const_iterator c_it = collision_model_->links_[l_i]->collision_array.begin(); c_it != collision_model_->links_[l_i]->collision_array.end(); c_it++)
		{
			if ((*c_it)->geometry->type == self_collision::Geometry::CONVEX)
			{
				convex_hull_vector_.push_back(*c_it);
			}
		}
	}

	//
	// prepare communication buffers
	//
	qhull_data_.qhulls.resize(convex_hull_vector_.size());
	for (int i=0; i<convex_hull_vector_.size(); i++)
	{
		qhull_data_.qhulls[i].points.resize(40);
		qhull_data_.qhulls[i].polygons.resize(40*6);
	}

	qhull_points_.point_lists.resize(convex_hull_vector_.size());
	qhull_points_sent_.point_lists.resize(convex_hull_vector_.size());
	for (int i=0; i<convex_hull_vector_.size(); i++)
	{
		qhull_points_.point_lists[i].points.resize(40);
		qhull_points_.point_lists[i].num_points = 0;
		qhull_points_sent_.point_lists[i].points.resize(40);
		qhull_points_sent_.point_lists[i].num_points = 0;
	}

	qhull_points_out_.setDataSample(qhull_points_);

	// joint_states
	joint_states_.name.resize(joints_count_);
	joint_states_.position.resize(joints_count_);
	joint_states_.velocity.resize(joints_count_);
	joint_states_.effort.resize(joints_count_);

	// visualization
	// draw all links' collision objects
	// iterate through all links
	for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
	{
		// iterate through collision objects
		for (self_collision::Link::VecPtrCollision::const_iterator c_it = collision_model_->links_[l_i]->collision_array.begin(); c_it != collision_model_->links_[l_i]->collision_array.end(); c_it++)
		{
			(*c_it)->geometry->addMarkers(markers_);
		}
	}

	// add markers for distances
	for (int i=0; i<prop_distances_count_; i++)
	{
		distances[i].addMarkers(markers_);
	}

	markers_out_.setDataSample(markers_);

	RTT::log(RTT::Info) << "prop_d0_: " << prop_d0_ << std::endl;
	RTT::log(RTT::Info) << "prop_distances_count_: " << prop_distances_count_ << std::endl;

	calculated_fk_.resize(collision_model_->link_count_);
	
	return true;
}

bool SelfCollisionAvoidance::startHook() {
	return true;
}

void SelfCollisionAvoidance::stopHook() {
}

void SelfCollisionAvoidance::updateHook() {
	joint_in_.read(joint_states_);
	if (joint_states_.name.size() != joints_count_)
	{
//		RTT::log(RTT::Error) << "wrong number of joints: " << joint_states_.name.size() << ", should be: " << joints_count_ << std::endl;
		std::cout << "wrong number of joints: " << joint_states_.name.size() << ", should be: " << joints_count_ << std::endl;
		stop();
		return;
	}

	qhull_data_in_.read(qhull_data_);

	// arrange the joint positions to their ids in the KDL tree
	for (int i=0; i<joint_states_.name.size(); i++)
	{
		joint_positions_by_index_[ joint_name_2_index_map_[joint_states_.name[i]] ] = joint_states_.position[i];
	}

	for (int l_i=0; l_i<collision_model_->link_count_; l_i++)
	{
		calculated_fk_[l_i] = false;
	}

	// calculate the forward kinematics for all links
	for (int fk_i = 0; fk_i < collision_model_->link_count_; fk_i++)
	{
		int l_i = fk_seq_[fk_i];
		int parent_i = collision_model_->links_[l_i]->parent_index_;
		double q_i = joint_positions_by_index_[l_i];

		if (collision_model_->links_[l_i]->kdl_segment_->segment.getName() != collision_model_->links_[l_i]->name)
		{
			std::cout << "inconsistent KDL tree" << std::endl;
			stop();
		}
		if (parent_i == -1)
		{
			transformations_by_index_[l_i] = collision_model_->links_[l_i]->kdl_segment_->segment.pose(q_i);
		}
		else
		{
			transformations_by_index_[l_i] = transformations_by_index_[parent_i] * collision_model_->links_[l_i]->kdl_segment_->segment.pose(q_i);
		}
		calculated_fk_[l_i] = true;
	}

	for (int l_i=0; l_i<collision_model_->link_count_; l_i++)
	{
		if (!calculated_fk_[l_i])
		{
//			RTT::log(RTT::Error) << "could not calculate fk: " << l_i << std::endl;
			std::cout << "could not calculate fk: " << l_i << std::endl;
			stop();
			return;
		}
	}

	//
	// update convex hulls
	//
	int convex_idx = 0;
	for (self_collision::Link::VecPtrCollision::iterator it = convex_hull_vector_.begin(); it != convex_hull_vector_.end(); it++, convex_idx++)
	{
			self_collision::Convex* convex = static_cast<self_collision::Convex*>((*it)->geometry.get());
			KDL::Frame &T_B_L = transformations_by_index_[(*it)->parent_->index_];

			qhull_points_.point_lists[convex_idx].num_points = 0;
			for (self_collision::Convex::ConvexPointsIdVector::iterator pt_it = convex->points_id_.begin(); pt_it != convex->points_id_.end(); pt_it++)
			{
				KDL::Frame &T_B_F = transformations_by_index_[pt_it->first];
				KDL::Frame T_E_F = (T_B_L * (*it)->origin).Inverse() * T_B_F;
				KDL::Vector pt = T_E_F * pt_it->second;
				qhull_points_.point_lists[convex_idx].points[qhull_points_.point_lists[convex_idx].num_points].x = pt.x();
				qhull_points_.point_lists[convex_idx].points[qhull_points_.point_lists[convex_idx].num_points].y = pt.y();
				qhull_points_.point_lists[convex_idx].points[qhull_points_.point_lists[convex_idx].num_points].z = pt.z();
				qhull_points_.point_lists[convex_idx].num_points++;
			}
			
			convex->updateConvex(qhull_data_.qhulls[convex_idx].num_points, qhull_data_.qhulls[convex_idx].points, qhull_data_.qhulls[convex_idx].num_planes, qhull_data_.qhulls[convex_idx].polygons);
			KDL::Vector center;
			for (int p_idx=0; p_idx<qhull_data_.qhulls[convex_idx].num_points; p_idx++)
			{
				center += KDL::Vector(qhull_data_.qhulls[convex_idx].points[p_idx].x, qhull_data_.qhulls[convex_idx].points[p_idx].y, qhull_data_.qhulls[convex_idx].points[p_idx].z);
			}
			center = 1.0/(double)qhull_data_.qhulls[convex_idx].num_points * center;

			double radius = 0.0;
			for (int p_idx=0; p_idx<qhull_data_.qhulls[convex_idx].num_points; p_idx++)
			{
				double d = (KDL::Vector(qhull_data_.qhulls[convex_idx].points[p_idx].x, qhull_data_.qhulls[convex_idx].points[p_idx].y, qhull_data_.qhulls[convex_idx].points[p_idx].z)-center).Norm();
				if (d > radius)
				{
					radius = d;
				}
			}
			convex->center_ = center;
			convex->radius_ = radius;
	}

	// check collisions between links
	int distances_count = 0;
	for (self_collision::CollisionModel::CollisionPairs::iterator it = collision_model_->enabled_collisions.begin(); it != collision_model_->enabled_collisions.end(); it++)
	{
		KDL::Vector d1, d2;
		KDL::Frame T_l1, T_l2;
		KDL::Frame &T_B_L1 = transformations_by_index_[it->first];
		KDL::Frame &T_B_L2 = transformations_by_index_[it->second];

		boost::shared_ptr< const self_collision::Link > link1 = collision_model_->getLink(it->first);
		boost::shared_ptr< const self_collision::Link > link2 = collision_model_->getLink(it->second);

		// iterate through collision objects of link1
		for (self_collision::Link::VecPtrCollision::const_iterator c_it1 = link1->collision_array.begin(); c_it1 != link1->collision_array.end(); c_it1++)
		{
			// iterate through collision objects of link2
			for (self_collision::Link::VecPtrCollision::const_iterator c_it2 = link2->collision_array.begin(); c_it2 != link2->collision_array.end(); c_it2++)
			{
				double dist = self_collision::CollisionModel::getDistance(*((*c_it1)->geometry.get()), T_B_L1 * (*c_it1)->origin, *((*c_it2)->geometry.get()), T_B_L2 * (*c_it2)->origin, d1, d2, prop_d0_);
				double dist2 = (d1-d2).Norm();
				if (dist-dist2 > 0.01 || dist-dist2 < -0.01)
				{
					// it is ok
				}

				if (dist < 0.0)
				{
//					RTT::log(RTT::Error) << "collision: " << it->first << " " << it->second << " " << (*c_it1)->geometry->type << " " << (*c_it2)->geometry->type << " " << dist << std::endl;
				}
				else if (dist < prop_d0_)
				{
					if (distances_count >= prop_distances_count_)
					{
//						RTT::log(RTT::Error) << "too many low distances" << std::endl;
					}
					else
					{
						distances[distances_count].i_ = it->first;
						distances[distances_count].j_ = it->second;
						distances[distances_count].d_ = dist;
						distances[distances_count].xi_ = T_B_L1.Inverse() * d1;
						distances[distances_count].xj_ = T_B_L1.Inverse() * d2;
						distances_count++;
					}
				}
			}
		}
	}

	// visualization
	// update collision model markers
	for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
	{
		// iterate through collision objects
		for (self_collision::Link::VecPtrCollision::const_iterator c_it = collision_model_->links_[l_i]->collision_array.begin(); c_it != collision_model_->links_[l_i]->collision_array.end(); c_it++)
		{
			(*c_it)->geometry->updateMarkers(markers_, transformations_by_index_[l_i] * (*c_it)->origin);
			if ( (transformations_by_index_[l_i] * (*c_it)->origin).p.Norm() < 0.001)
			{
				std::cout << "wrong transformation: " << l_i << std::endl;
				stop();
			}
		}
	}

	// update distance markers
	for (int i=0; i<prop_distances_count_; i++)
	{
		if (i >= distances_count)
		{
			distances[i].i_ = 0;
			distances[i].j_ = 0;
			distances[i].xi_ = KDL::Vector();
			distances[i].xj_ = KDL::Vector();
		}
		distances[i].updateMarkers(markers_, transformations_by_index_[distances[i].i_]);
	}

	markers_out_.write(markers_);

	if (isQhullUpdateNeeded())
	{
		// check if we want to send the qhull points
		qhull_points_out_.write(qhull_points_);

		for (int i=0; i<qhull_points_.point_lists.size(); i++)
		{
			qhull_points_sent_.point_lists[i].num_points = qhull_points_.point_lists[i].num_points;
			for (int p_idx=0; p_idx<qhull_points_.point_lists[i].num_points; p_idx++)
			{
				qhull_points_sent_.point_lists[i].points[p_idx] = qhull_points_.point_lists[i].points[p_idx];
			}
		}
	}
}

bool SelfCollisionAvoidance::isQhullUpdateNeeded()
{
	for (int convex_idx=0; convex_idx<qhull_data_.qhulls.size(); convex_idx++)
	{	
		if ( qhull_data_.qhulls[convex_idx].num_points < 4 )
		{
			return true;
		}
	}

	if (qhull_points_.point_lists.size() != qhull_points_sent_.point_lists.size())
	{
		return true;
	}
	for (int i=0; i<qhull_points_.point_lists.size(); i++)
	{
		if ( qhull_points_.point_lists[i].num_points != qhull_points_sent_.point_lists[i].num_points )
		{
			return true;
		}
		for (int p_idx=0; p_idx<qhull_points_.point_lists[i].num_points; p_idx++)
		{
			double dx = qhull_points_.point_lists[i].points[p_idx].x - qhull_points_sent_.point_lists[i].points[p_idx].x;
			double dy = qhull_points_.point_lists[i].points[p_idx].y - qhull_points_sent_.point_lists[i].points[p_idx].y;
			double dz = qhull_points_.point_lists[i].points[p_idx].z - qhull_points_sent_.point_lists[i].points[p_idx].z;
			if ( dx > 0.001 || dx < -0.001 || dy > 0.001 || dy < -0.001 || dz > 0.001 || dz < -0.001)
			{
				return true;
			}
		}
	}
	return false;
}

ORO_CREATE_COMPONENT(SelfCollisionAvoidance)

