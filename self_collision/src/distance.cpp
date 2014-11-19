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

#include "distance.h"

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
	marker3.color.r = 1;
	marker3.color.g = 0;
	marker3.color.b = 0;
	marker_array.markers.push_back(marker3);
}

void Distance::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &T_B_Ti, const KDL::Frame &T_B_Tj)
{
	KDL::Vector pos1 = T_B_Ti * xi_Ti_;
	KDL::Vector pos2 = T_B_Tj * xj_Tj_;

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

void VectorVis::addMarkers(visualization_msgs::MarkerArray &marker_array)
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
	marker.type = visualization_msgs::Marker::ARROW;
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
	marker.scale.y = 0.02;
	marker.scale.z = 0.0;
	marker.color.a = 1.0;
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0;
	marker_array.markers.push_back(marker);
}

void VectorVis::updateMarkers(visualization_msgs::MarkerArray &marker_array, KDL::Vector x1, KDL::Vector x2)
{
	marker_array.markers[marker_id_].header.stamp = ros::Time();
	marker_array.markers[marker_id_].points.at(0).x = x1.x();
	marker_array.markers[marker_id_].points.at(0).y = x1.y();
	marker_array.markers[marker_id_].points.at(0).z = x1.z();
	marker_array.markers[marker_id_].points.at(1).x = x2.x();
	marker_array.markers[marker_id_].points.at(1).y = x2.y();
	marker_array.markers[marker_id_].points.at(1).z = x2.z();
}


