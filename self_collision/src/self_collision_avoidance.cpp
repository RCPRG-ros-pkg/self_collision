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

#include <kdl_parser/kdl_parser.hpp>

SelfCollisionAvoidance::SelfCollisionAvoidance(const std::string& name) :
	RTT::TaskContext(name),
	joints_count_(0)
{
	this->addProperty("robot_description", prop_robot_description_);
	this->addProperty("robot_semantic_description", prop_robot_semantic_description_);
	this->addProperty("distances_count", prop_distances_count_);
	this->addProperty("d0", prop_d0_);
	this->addProperty("joint_position_sequence", prop_joint_position_sequence_);
	this->addProperty("ctrl_joint_position_sequence", prop_ctrl_joint_position_sequence_);
	this->addProperty("ros_joint_states_names", prop_ros_joint_states_names_);

	this->ports()->addPort("JointPositionIn", port_joint_in_);
	this->ports()->addPort("CtrlJointPositionIn", port_ctrl_joint_in_);
	this->ports()->addPort("GripperJointPosition", port_gripper_joint_in_);
	this->ports()->addPort("MassMatrix", port_mass_matrix_);

	this->ports()->addPort("dbg_markers", markers_out_);

	this->ports()->addPort("QhullDataIn", qhull_data_in_);
	this->ports()->addPort("QhullPointsOut", qhull_points_out_);
}

SelfCollisionAvoidance::~SelfCollisionAvoidance() {
}

//int SelfCollisionAvoidance::JntToJac(const KDL::JntArray& q_in, KDL::Jacobian& jac, const std::string& segmentname) {
int SelfCollisionAvoidance::JntToJac(KDL::Jacobian& jac, int link_index, const KDL::Vector &x) {
//joint_positions_by_index_[l_i];
	//First we check all the sizes:
//	if (q_in.rows() != tree.getNrOfJoints() || jac.columns() != tree.getNrOfJoints())
	if (jac.columns() != collision_model_->link_count_)
		return -1;
	//Lets search the tree-element
//	SegmentMap::const_iterator it = tree.getSegments().find(segmentname);
	//If segmentname is not inside the tree, back out:
	//Let's make the jacobian zero:
	jac.data.setZero();
//	KDL::SetToZero(jac);
//	SegmentMap::const_iterator root = tree.getRootSegment();
//	KDL::Frame T_total = KDL::Frame::Identity();
	KDL::Frame T_total(x);
	int l_index = link_index;
	//Lets recursively iterate until we are in the root segment
	while (l_index != collision_model_->root_index_) {
		//get the corresponding q_nr for this TreeElement:
//		unsigned int q_nr = GetTreeElementQNr(it->second);
		//get the pose of the segment:
//		Frame T_local = GetTreeElementSegment(it->second).pose(q_in(q_nr));
		KDL::Frame T_local = collision_model_->links_[l_index]->kdl_segment_->segment.pose(joint_positions_by_index_[l_index]);

		//calculate new T_end:
		T_total = T_local * T_total;
		//get the twist of the segment:
//		if (GetTreeElementSegment(it->second).getJoint().getType() != Joint::None) {
//		Twist t_local = GetTreeElementSegment(it->second).twist(q_in(q_nr), 1.0);
		KDL::Twist t_local = collision_model_->links_[l_index]->kdl_segment_->segment.twist(joint_positions_by_index_[l_index], 1.0);
		//transform the endpoint of the local twist to the global endpoint:
		t_local = t_local.RefPoint(T_total.p - T_local.p);
		//transform the base of the twist to the endpoint
		t_local = T_total.M.Inverse(t_local);
		//store the twist in the jacobian:
		//jac.setColumn(q_nr,t_local);
		jac.setColumn(l_index,t_local);
//		}//endif
		//goto the parent
//		it = GetTreeElementParent(it->second);
		l_index = collision_model_->links_[l_index]->parent_index_;
	}//endwhile
	//Change the base of the complete jacobian from the endpoint to the base
	changeBase(jac, T_total.M, jac);
	return 0;
}//end JntToJac

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


	// prepare vector of mimic joints
	typedef std::map< std::string,boost::shared_ptr< urdf::Joint > > JointMap;
	for (JointMap::iterator it = robot_model_.joints_.begin(); it != robot_model_.joints_.end(); it++)
	{
		// name: it->first
		if (it->second->mimic.get() != NULL)
		{
			RTT::log(RTT::Info) << "added mimic joint: " << it->first << " = " << it->second->mimic->joint_name << " * " << it->second->mimic->multiplier << " + " << it->second->mimic->offset << std::endl;
			mimic_joints_.push_back( std::make_pair<std::string, boost::shared_ptr< urdf::JointMimic > >(it->first, it->second->mimic) );
		}
	}

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
			if (collision_model_->root_index_ == -1)
			{
				collision_model_->root_index_ = l_i;
			}
			else
			{
				RTT::log(RTT::Error) << "link: " << collision_model_->links_[l_i]->name << " is has no parent - it is the second root" << std::endl;
			}
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
	time_since_last_qhull_update_ = 1000;

	port_joint_in_.getDataSample(jnt_pos_);
	RTT::log(RTT::Info) << "jnt_pos_.size(): " << jnt_pos_.size() << std::endl;
	if (jnt_pos_.size() != prop_joint_position_sequence_.size())
	{
		RTT::log(RTT::Error) << "jnt_pos_.size() != prop_joint_position_sequence_.size(): " << jnt_pos_.size() << " != " << prop_joint_position_sequence_.size() << std::endl;
		return false;
	}

	for (int i=0; i<prop_joint_position_sequence_.size(); i++)
	{
		RTT::log(RTT::Info) << "joint_position_sequence: " << i << ": <" << prop_joint_position_sequence_[i] << ">" << std::endl;
	}

	port_ctrl_joint_in_.getDataSample(ctrl_jnt_pos_);
	RTT::log(RTT::Info) << "ctrl_jnt_pos_.size(): " << ctrl_jnt_pos_.size() << std::endl;
	if (ctrl_jnt_pos_.size() != prop_ctrl_joint_position_sequence_.size())
	{
		RTT::log(RTT::Error) << "ctrl_jnt_pos_.size() != prop_ctrl_joint_position_sequence_.size(): " << ctrl_jnt_pos_.size() << " != " << prop_ctrl_joint_position_sequence_.size() << std::endl;
		return false;
	}

	for (int i=0; i<prop_ctrl_joint_position_sequence_.size(); i++)
	{
		RTT::log(RTT::Info) << "joint_position_sequence: " << i << ": <" << prop_ctrl_joint_position_sequence_[i] << ">" << std::endl;
	}

	gripper_joint_states_.name.resize(joints_count_);
	gripper_joint_states_.position.resize(joints_count_);
	gripper_joint_states_.velocity.resize(joints_count_);
	gripper_joint_states_.effort.resize(joints_count_);

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

	Fmax_ = 10.0;
	Frep_mult_ = Fmax_ / (prop_d0_ * prop_d0_);

	// mass matrix
	port_mass_matrix_.getDataSample(mass_matrix_);
	if (mass_matrix_.rows() != ctrl_jnt_pos_.size())
	{
		RTT::log(RTT::Error) << "mass_matrix_.rows() != ctrl_jnt_pos_.size(): " << mass_matrix_.rows() << " != " << ctrl_jnt_pos_.size() << RTT::endlog();
		return false;
	}
	if (mass_matrix_.cols() != ctrl_jnt_pos_.size())
	{
		RTT::log(RTT::Error) << "mass_matrix_.cols() != ctrl_jnt_pos_.size(): " << mass_matrix_.cols() << " != " << ctrl_jnt_pos_.size() << RTT::endlog();
		return false;
	}

	lu_ = Eigen::PartialPivLU<Eigen::MatrixXd>(ctrl_jnt_pos_.size());

	Jxi_.resize(3, ctrl_jnt_pos_.size());	// 3 rows	ctrl_jnt_pos_.size() columns
	Jdi_.resize(1, 3);			// 1 row	3 columns
	Ji_.resize(1, ctrl_jnt_pos_.size());
	JiT_.resize(ctrl_jnt_pos_.size(),1);
	tmp1n_.resize(1, ctrl_jnt_pos_.size());
	mass_matrix_inv_.resize(ctrl_jnt_pos_.size(), ctrl_jnt_pos_.size());	// ctrl_jnt_pos_.size() rows	ctrl_jnt_pos_.size() columns
	Mdi_inv_.resize(1,1);
	Mdi_.resize(1,1);
	Kdij_.resize(2,2);

	es_ = Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd>(2);

	// tests
	test_jnt_idx_ = joint_name_2_index_map_["right_arm_3_joint"];
	std::cout << "right_arm_3_joint idx: " << test_jnt_idx_ << std::endl;
	test_vec_.resize(200);
	for (int i=0; i<test_vec_.size(); i++)
	{
		test_vec_[i].addMarkers(markers_);
	}

	return true;
}

bool SelfCollisionAvoidance::startHook() {
	return true;
}

void SelfCollisionAvoidance::stopHook() {
}

void SelfCollisionAvoidance::updateHook() {
	port_mass_matrix_.read(mass_matrix_);

	port_joint_in_.read(jnt_pos_);
	for (int i=0; i<prop_joint_position_sequence_.size(); i++)
	{
		joint_positions_by_index_[ joint_name_2_index_map_[prop_joint_position_sequence_[i]] ] = jnt_pos_[i];
	}

	port_ctrl_joint_in_.read(ctrl_jnt_pos_);
	for (int i=0; i<prop_ctrl_joint_position_sequence_.size(); i++)
	{
		joint_positions_by_index_[ joint_name_2_index_map_[prop_ctrl_joint_position_sequence_[i]] ] = ctrl_jnt_pos_[i];
	}

	port_gripper_joint_in_.read(gripper_joint_states_);

	qhull_data_in_.read(qhull_data_);

	for (int i=0; i<gripper_joint_states_.name.size(); i++)
	{
		for (int j=0; j<prop_ros_joint_states_names_.size(); j++)
		{
			if (gripper_joint_states_.name[i] == prop_ros_joint_states_names_[j])
			{
				joint_positions_by_index_[ joint_name_2_index_map_[gripper_joint_states_.name[i]] ] = gripper_joint_states_.position[i];
				break;
			}
		}
	}

	// calculate mimic joints' positions
	for (int i=0; i<mimic_joints_.size(); i++)
	{
		int joint_idx = joint_name_2_index_map_[ mimic_joints_[i].first ];
		int mimic_joint_idx = joint_name_2_index_map_[ mimic_joints_[i].second->joint_name ];
		joint_positions_by_index_[joint_idx] = joint_positions_by_index_[mimic_joint_idx] * mimic_joints_[i].second->multiplier + mimic_joints_[i].second->offset;
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
			RTT::log(RTT::Error) << "Inconsistent KDL tree" << RTT::endlog();
			error();
			return;
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
			RTT::log(RTT::Error) << "Could not calculate fk" << RTT::endlog();
			error();
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
					RTT::log(RTT::Error) << "Collision detected" << RTT::endlog();
					error();
					return;
				}
				else if (dist < prop_d0_)
				{
					if (distances_count >= prop_distances_count_)
					{
						RTT::log(RTT::Error) << "Too many collision pairs" << RTT::endlog();
						error();
						return;
					}
					else
					{
						distances[distances_count].i_ = it->first;
						distances[distances_count].j_ = it->second;
						distances[distances_count].d_ = dist;
						distances[distances_count].xi_Ti_ = T_B_L1.Inverse() * d1;
						distances[distances_count].xj_Tj_ = T_B_L2.Inverse() * d2;
						distances_count++;
					}
				}
			}
		}
	}

	// invert the mass matrix
	lu_.compute(mass_matrix_);
	mass_matrix_inv_ = lu_.inverse();

	int test_vec_idx = 0;
	// calculate jacobians
	for (int i=0; i<distances_count; i++)
	{
		KDL::Jacobian Jxi(collision_model_->link_count_);
		KDL::Jacobian Jxj(collision_model_->link_count_);

		int ret = JntToJac(Jxi, distances[i].i_, distances[i].xi_Ti_);
		if (ret != 0)
		{
			RTT::log(RTT::Error) << "Jacobian calculation error" << RTT::endlog();
			error();
			return;
		}

//		KDL::Vector xj = transformations_by_index_[distances[i].j_].Inverse() * transformations_by_index_[distances[i].i_] * distances[i].xj_;
		ret = JntToJac(Jxj, distances[i].j_, distances[i].xj_Tj_);
		if (ret != 0)
		{
			RTT::log(RTT::Error) << "Jacobian calculation error" << RTT::endlog();
			error();
			return;
		}

		KDL::Vector p1 = transformations_by_index_[distances[i].i_] * distances[i].xi_Ti_;
		KDL::Vector p2 = transformations_by_index_[distances[i].j_] * distances[i].xj_Tj_;
		// test: visualization
/*		KDL::Twist t1 = Jxi.getColumn(test_jnt_idx_) * (-0.5);
		test_vec_[test_vec_idx].updateMarkers(markers_, p1, p1 + t1.vel);
		test_vec_idx++;

		KDL::Twist t2 = Jxj.getColumn(test_jnt_idx_) * (-0.5);
		test_vec_[test_vec_idx].updateMarkers(markers_, p2, p2 + t2.vel);
		test_vec_idx++;
*/
		// check if the distance returned from distance algorithm is equal to |p1-p2|
		double d_ij = (p1-p2).Norm();
		if (d_ij-distances[i].d_ > 0.0001 || d_ij-distances[i].d_ < -0.0001)
		{
			RTT::log(RTT::Error) << "ERROR: distances do not match: " << d_ij << " " << distances[i].d_ << RTT::endlog();
			error();
			return;
		}

		if (distances[i].d_ > prop_d0_)
		{
			RTT::log(RTT::Error) << "ERROR: distances is too big: " << distances[i].d_ << RTT::endlog();
			error();
			return;
		}

		// eq. 8
		double Frep_ij = Frep_mult_ * (distances[i].d_ - prop_d0_) * (distances[i].d_ - prop_d0_);

		// TODO: calculate jacobian for point
		KDL::Vector xj_Ti = transformations_by_index_[distances[i].i_].Inverse() * transformations_by_index_[distances[i].j_] * distances[i].xj_Tj_;

		// eq. 4
		KDL::Vector e_i = xj_Ti - distances[i].xi_Ti_;
		e_i.Normalize();

		// eq. 15
		Jdi_(0,0) = e_i.x();
		Jdi_(0,1) = e_i.y();
		Jdi_(0,2) = e_i.z();

		// rewrite the linear part of the jacobian
		for (int j=0; j<prop_ctrl_joint_position_sequence_.size(); j++)
		{
			KDL::Vector v = Jxi.getColumn( joint_name_2_index_map_[prop_ctrl_joint_position_sequence_[i]] ).vel;
			Jxi_(0, j) = v.x();
			Jxi_(1, j) = v.y();
			Jxi_(2, j) = v.z();
		}

		// eq. 16
		Ji_.noalias() = Jdi_ * Jxi_;			// (1 x |ctrl_jnt_pos_|) = (1 x 3) * (3 x |ctrl_jnt_pos_|)
		JiT_ = Ji_.transpose();

		// eq. 18
		tmp1n_.noalias() = Ji_ * mass_matrix_inv_;	// (1 x |ctrl_jnt_pos_|) = (1 x |ctrl_jnt_pos_|) * (|ctrl_jnt_pos_| x |ctrl_jnt_pos_|)
		Mdi_inv_.noalias() = tmp1n_ * JiT_;			// (1 x 1) = (1 x |ctrl_jnt_pos_|) * (|ctrl_jnt_pos_| x 1)
		Mdi_(0,0) = 1.0 / Mdi_inv_(0,0);

		// eq. 21
		double d2Vrep = -2.0 * Frep_mult_ * (distances[i].d_ - prop_d0_);
		Kdij_(0,0) = d2Vrep;
		Kdij_(1,1) = d2Vrep;
		Kdij_(0,1) = -d2Vrep;
		Kdij_(1,0) = -d2Vrep;


// double diagonalization method
//    tmpNK_.noalias() = J * Mi;
//    A.noalias() = tmpNK_ * JT;
//    luKK_.compute(A);
//    A = luKK_.inverse();

//    tmpKK_ = Kc.asDiagonal();
//    UNRESTRICT_ALLOC;
//    es_.compute(tmpKK_, A);
//    RESTRICT_ALLOC;
//    K0 = es_.eigenvalues();
//    luKK_.compute(es_.eigenvectors());
//    Q = luKK_.inverse();

//    tmpKK_ = Dxi.asDiagonal();
//    Dc.noalias() = Q.transpose() * tmpKK_;
//    tmpKK_ = K0.cwiseSqrt().asDiagonal();
//    tmpKK2_.noalias() = Dc *  tmpKK_;
//    Dc.noalias() = tmpKK2_ * Q;
//    tmpK_.noalias() = J * joint_velocity_;
//    F.noalias() = Dc * tmpK_;
//    joint_torque_command_.noalias() -= JT * F;


	}
	for (; test_vec_idx<test_vec_.size(); test_vec_idx++)
	{
		test_vec_[test_vec_idx].updateMarkers(markers_, KDL::Vector(), KDL::Vector());
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
				RTT::log(RTT::Error) << "wrong transformation: " << l_i << RTT::endlog();
				error();
				return;
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
			distances[i].xi_Ti_ = KDL::Vector();
			distances[i].xj_Tj_ = KDL::Vector();
		}
		distances[i].updateMarkers(markers_, transformations_by_index_[distances[i].i_], transformations_by_index_[distances[i].j_]);
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
		time_since_last_qhull_update_ = 0;
	}
	if (time_since_last_qhull_update_ < 1000)
	{
		time_since_last_qhull_update_++;
	}
}

bool SelfCollisionAvoidance::isQhullUpdateNeeded()
{
	// qhull update rate limit
	if (time_since_last_qhull_update_ <= 10)
	{
		return false;
	}

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

