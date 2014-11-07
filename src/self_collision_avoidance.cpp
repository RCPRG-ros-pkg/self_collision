// Copyright 2014 WUT
/*
 * self_collision_avoidance.cpp
 *
 *  Created on: 06 nov 2014
 *      Author: dseredyn
 */

#include "self_collision_avoidance.h"

//#include <string>

//#include "rtt_rosclock/rtt_rosclock.h"
//#include "Eigen/Geometry"
#include "rtt/Component.hpp"

#include "urdf/model.h"
#include <kdl_parser/kdl_parser.hpp>
#include "urdf_collision_parser.h"

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

	marker_array.markers[marker_id_].header.stamp = ros::Time();
	marker_array.markers[marker_id_].points[0].x = pos1.x();
	marker_array.markers[marker_id_].points[0].y = pos1.y();
	marker_array.markers[marker_id_].points[0].z = pos1.z();
	marker_array.markers[marker_id_].points[1].x = pos2.x();
	marker_array.markers[marker_id_].points[1].y = pos2.y();
	marker_array.markers[marker_id_].points[1].z = pos2.z();

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
}

SelfCollisionAvoidance::~SelfCollisionAvoidance() {
}

bool SelfCollisionAvoidance::configureHook() {

	// read KDL::Tree form the robot_description
	KDL::Tree robot_tree;
	if (!kdl_parser::treeFromString(prop_robot_description_, robot_tree))
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
		collision_model_->links_[l_i]->kdl_segment_ = &(robot_tree.getSegment(collision_model_->links_[l_i]->name)->second);
	}

	// prepare the map of joint states
	for (KDL::SegmentMap::const_iterator seg_it = robot_tree.getSegments().begin(); seg_it != robot_tree.getSegments().end(); seg_it++)
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

	//
	// prepare communication buffers
	//

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

//			m_id = (*c_it)->geometry->publishMarker(vis_pub, m_id, transformation_map[l_i] * (*c_it)->origin);
		}
	}

	// add markers for distances
	for (int i=0; i<prop_distances_count_; i++)
	{
		distances[i].addMarkers(markers_);
		RTT::log(RTT::Info) << "distance[" << i << "].marker_id_ == " << distances[i].marker_id_ << std::endl;
	}

	// draw all low distances
//	for (int l_d = 0; l_d < distances_count; l_d++)
//	{
//		KDL::Frame &T_B_L1 = transformation_map[distances[l_d].i_];
//		m_id = publishLineMarker(vis_pub, m_id, T_B_L1 * distances[l_d].xi_, T_B_L1 * distances[l_d].xj_, 1, 0, 0);
//	}

	RTT::log(RTT::Info) << "prop_d0_: " << prop_d0_ << std::endl;
	RTT::log(RTT::Info) << "prop_distances_count_: " << prop_distances_count_ << std::endl;
		

//	port_JointPosition.setDataSample(jnt_pos_);
	return true;
}

bool SelfCollisionAvoidance::startHook() {
	return true;
/*
  if (activate_pose_init_property_) {
    setpoint_ = init_setpoint_property_;
  } else {
    if (port_cartesian_position_.read(setpoint_) == RTT::NoData) {
      return false;
    }
  }
  last_point_not_set_ = false;
  trajectory_active_ = false;
  return true;*/
}

void SelfCollisionAvoidance::updateHook() {
	joint_in_.read(joint_states_);

	if (joint_states_.name.size() != joints_count_)
	{
		RTT::log(RTT::Error) << "wrong number of joints: " << joint_states_.name.size() << ", should be: " << joints_count_ << std::endl;
		return;
	}

	// arrange the joint positions to their ids in the KDL tree
	for (int i=0; i<joint_states_.name.size(); i++)
	{
		joint_positions_by_index_[ joint_name_2_index_map_[joint_states_.name[i]] ] = joint_states_.position[i];
	}

	// calculate the forward kinematics for all links
	for (int fk_i = 0; fk_i < collision_model_->link_count_; fk_i++)
	{
		int l_i = fk_seq_[fk_i];
		int parent_i = collision_model_->links_[l_i]->parent_index_;
		double q_i = joint_positions_by_index_[l_i];
		if (parent_i == -1)
		{
			transformations_by_index_[l_i] = collision_model_->links_[l_i]->kdl_segment_->segment.pose(q_i);
		}
		else
		{
			transformations_by_index_[l_i] = transformations_by_index_[parent_i] * collision_model_->links_[l_i]->kdl_segment_->segment.pose(q_i);
		}
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
		}
	}

	if (distances_count > 0)
	{
//		RTT::log(RTT::Info) << "distances: " << distances_count << std::endl;
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



/*  if (port_trajectory_.read(trajectory_) == RTT::NewData) {
    trajectory_ptr_ = 0;
    old_point_ = setpoint_;
    last_point_not_set_ = true;
    trajectory_active_ = true;
  //  std::cout << std::endl<< "CartesianInterpolator new trj" << std::endl<< std::endl<< std::endl;
  }
  //std::cout << "CartesianInterpolator" << std::endl;
  ros::Time now = rtt_rosclock::host_now();
  if (trajectory_active_ && trajectory_ && (trajectory_->header.stamp < now)) {
    for (; trajectory_ptr_ < trajectory_->points.size(); trajectory_ptr_++) {
      ros::Time trj_time = trajectory_->header.stamp
          + trajectory_->points[trajectory_ptr_].time_from_start;
      if (trj_time > now) {
        break;
      }
    }

    if (trajectory_ptr_ < trajectory_->points.size()) {
      if (trajectory_ptr_ == 0) {
        cartesian_trajectory_msgs::CartesianTrajectoryPoint p0;
        p0.time_from_start.fromSec(0.0);
        p0.pose = old_point_;
        setpoint_ = interpolate(p0, trajectory_->points[trajectory_ptr_], now);
      } else {
        setpoint_ = interpolate(trajectory_->points[trajectory_ptr_ - 1],
                                trajectory_->points[trajectory_ptr_], now);
      }
    } else if (last_point_not_set_) {
      setpoint_ = trajectory_->points[trajectory_->points.size() - 1].pose;
      last_point_not_set_ = false;
    }
  }
  port_cartesian_command_.write(setpoint_);
*/
}

ORO_CREATE_COMPONENT(SelfCollisionAvoidance)

