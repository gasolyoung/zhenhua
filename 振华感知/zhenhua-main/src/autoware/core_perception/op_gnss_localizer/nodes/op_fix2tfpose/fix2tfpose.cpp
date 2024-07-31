/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include "autoware_msgs/LaneArray.h"
#include <op_planner/RoadNetwork.h>
#include <op_planner/MappingHelpers.h>
#include <op_planner/PlanningHelpers.h>
#include <op_planner/KmlMapLoader.h>

#include <iostream>
#include <gnss/geo_pos_conv.hpp>

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher;
static ros::Publisher ndt_pose_pub;
static ros::Publisher estimate_twist_pub;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw = 0;
// true if position history is long enough to compute orientation
static bool _orientation_ready = false;
static bool bFirstGPSPoint = true;
geometry_msgs::PoseStamped g_path_initial_point;
static int _plane;

	PlannerHNS::MAP_SOURCE_TYPE g_MapType = PlannerHNS::MAP_KML_FILE;
	std::string g_MapPath;
	PlannerHNS::RoadNetwork g_Map;
	bool g_bMap = false;
  bool g_bFirstCall = false;

nav_msgs::Path _global_waypoints;

static void PathCallBack(const nav_msgs::PathConstPtr& msg)
{

if(_orientation_ready) return;

if(msg->poses.size() > 2)
{
    geometry_msgs::PoseStamped p1 = msg->poses.at(0);
    geometry_msgs::PoseStamped p2 = msg->poses.at(1);
    g_path_initial_point = p1;
    yaw = atan2(p2.pose.position.y - p1.pose.position.y, p2.pose.position.x - p1.pose.position.x);
    g_path_initial_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  _orientation_ready = true;
  double d = hypot(p2.pose.position.y - p1.pose.position.y, p2.pose.position.x - p1.pose.position.x);
  std::cout << "Find Angle From Path : " << yaw << ", Size: " << msg->poses.size() << ", Distance: " << d << std::endl;
}

}

	void ReadCommonParams()
	{
		ros::NodeHandle _nh("~");
		int iSource = 0;
		_nh.getParam("mapSource" , iSource);
		_nh.getParam("mapFileName" , g_MapPath);

		if(iSource == 0)
		{
			g_MapType = PlannerHNS::MAP_AUTOWARE;			
		}
		else if (iSource == 1)
		{
			g_MapType = PlannerHNS::MAP_FOLDER;			
		}
		else if(iSource == 2)
		{
			g_MapType = PlannerHNS::MAP_KML_FILE;
		}
		else if(iSource == 3)
		{
			g_MapType = PlannerHNS::MAP_LANELET_2;			
		}
		else if(iSource == 4)
		{
			g_MapType = PlannerHNS::MAP_KML_FILE_NAME;
		}
    else
    {
      g_MapType = PlannerHNS::MAP_NEW;
    }		
	}
	void LoadMap(const std::string& file_name)
	{
		PlannerHNS::KmlMapLoader kml_loader;
		kml_loader.LoadKML(file_name, g_Map);
		PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(g_Map);
	}

void callbackGetGlobalPlannerPath(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(_orientation_ready) return;

	if(msg->lanes.size() > 0 && msg->lanes.at(0).waypoints.size() > 2)
	{
		geometry_msgs::PoseStamped p1 = msg->lanes.at(0).waypoints.at(0).pose;
		geometry_msgs::PoseStamped p2 = msg->lanes.at(0).waypoints.at(1).pose;
		g_path_initial_point = p1;
		yaw = atan2(p2.pose.position.y - p1.pose.position.y, p2.pose.position.x - p1.pose.position.x);
		g_path_initial_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	  _orientation_ready = true;
	  double d = hypot(p2.pose.position.y - p1.pose.position.y, p2.pose.position.x - p1.pose.position.x);
	  std::cout << "Find Angle From Path : " << yaw << ", Size: " << msg->lanes.at(0).waypoints.size() << ", Distance: " << d << std::endl;
	}
}

static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
  geo_pos_conv geo;
  geo.set_plane(_plane);
  geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);
  

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;


  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/map";
  pose.pose.position.x = geo.y();
  pose.pose.position.y = geo.x();
  pose.pose.position.z = geo.z();

  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  if(!bFirstGPSPoint)
  {
	  double distance = hypot(_prev_pose.pose.position.y -  pose.pose.position.y, _prev_pose.pose.position.x -  pose.pose.position.x);

	   if (distance > 2.5)
	   {
		 yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
		 _prev_pose = pose;
		 //std::cout << "GNSS Localizer, Yaw: " << yaw << ", distance: " << distance << std::endl;
	   }
  }
  else
  {
	  bFirstGPSPoint = false;
	  _prev_pose = pose;
  }

  if(g_bMap && !g_bFirstCall)
  {
    PlannerHNS::WayPoint p(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0);
    PlannerHNS::Lane* pLane =  PlannerHNS::MappingHelpers::GetClosestLaneFromMap(p, g_Map, 20, false);
    PlannerHNS::WayPoint pWP;
    double d_to_closest = 100;

    if(pLane != nullptr)
    {
      PlannerHNS::RelativeInfo info;
			if(PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(pLane->points, p, info))
      {
        d_to_closest = fabs(info.perp_distance);
        pWP = info.perp_point;
      }
    }
      std::cout << " >>>>> Trying to Get Closest Point : " << d_to_closest << ", GPS_X: " << p.pos.x 
        << ", GPS_Y: " << p.pos.y << ", MapX:" << pWP.pos.x << ", MapY: " << pWP.pos.y << std::endl;

    if(d_to_closest <= 2.0)
    {
      yaw = pWP.pos.a;
      _orientation_ready = true;
      g_bFirstCall = true;
    }
  }

  if (_orientation_ready)
  {
	_quat = tf::createQuaternionMsgFromYaw(yaw);
    pose.pose.orientation = _quat;
    pose_publisher.publish(pose);
    stat_publisher.publish(gnss_stat_msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "/map", "/gps"));


//    geometry_msgs::PoseStamped ndt_pose_msg = pose;
//    ndt_pose_pub.publish(ndt_pose_msg);
//    static geometry_msgs::TwistStamped estimate_twist_msg;
//    estimate_twist_msg.header.stamp = ros::Time::now();
//    estimate_twist_msg.header.frame_id = "/base_link";
//    estimate_twist_msg.twist.linear.x = 0;
//    estimate_twist_msg.twist.linear.y = 0.0;
//    estimate_twist_msg.twist.linear.z = 0.0;
//    estimate_twist_msg.twist.angular.x = 0.0;
//    estimate_twist_msg.twist.angular.y = 0.0;
//    estimate_twist_msg.twist.angular.z = yaw;
//    estimate_twist_pub.publish(estimate_twist_msg);
//    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "/map", "/base_link"));
  }
}

	void kmlMapFileNameCallback(const std_msgs::String& file_name)
	{
		LoadMap(file_name.data);
	}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op_fix2tfpose");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("plane", _plane);  
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
  stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 10);

  ReadCommonParams();

//  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 10);
//  estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 10);

  ros::Subscriber gnss_pose_subscriber = nh.subscribe("fix", 10, GNSSCallback);
  //ros::Subscriber path_subscriber = nh.subscribe("carla/ego_vehicle/waypoints", 10, PathCallBack);
  ros::Subscriber sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &callbackGetGlobalPlannerPath);
  
  	if(g_MapType == PlannerHNS::MAP_KML_FILE_NAME)
		{
			ros::Subscriber sub_map_file_name = nh.subscribe("/assure_kml_map_file_name", 1, &kmlMapFileNameCallback);
		}

  while(g_MapType == PlannerHNS::MAP_KML_FILE && !g_bMap)
  {
    std::cout << "Trying to Load the map to Initialize GPS Orientation " << std::endl;
    ReadCommonParams();      
    LoadMap(g_MapPath);

    if(g_Map.roadSegments.size() > 0)
    {
      g_bMap = true;        
    }
  }
  ros::spin();
  return 0;
}
