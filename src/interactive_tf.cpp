/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Intelligent Robotics Core S.L.
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
 *   * Neither the name of Intelligent Robotics Core nor the names of its
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
 **********************************************************************/

/* Author: Daniel Puerto Núñez - dpuertonunez@gmail.com */

#include <boost/bind.hpp>
#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <tf/transform_datatypes.h>


class InteractiveTf
{
  ros::NodeHandle nh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  void processFeedback(unsigned ind, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  visualization_msgs::InteractiveMarker int_marker_;

  geometry_msgs::Pose pose_;
  tf::TransformBroadcaster br_;
  std::string parent_frame_;
  std::string frame_;
  void updateTf(int, const ros::TimerEvent& event);
  ros::Timer tf_timer_;

  bool changed_;


public:
  InteractiveTf();
  ~InteractiveTf();

  bool dumpCB();
};

InteractiveTf::InteractiveTf() :
  nh_(),
  parent_frame_("map"),
  frame_("interactive_tf"),
  changed_(false)
{
  server_.reset(new interactive_markers::InteractiveMarkerServer("/"+ros::this_node::getName()+"/interactive_tf"));
  std::vector<double> pos;
  std::vector<double> ori;

  if (nh_.hasParam("/cameras/"+ros::this_node::getName()+"/position"))
  {
     nh_.getParam("/cameras/"+ros::this_node::getName()+"/position", pos);

     pose_.position.x = pos[0];
     pose_.position.y = pos[1];
     pose_.position.z = pos[2];
  } else
     ROS_WARN("No hay posicion");

  if (nh_.hasParam("/cameras/"+ros::this_node::getName()+"/orientation"))
  {
     nh_.getParam("/cameras/"+ros::this_node::getName()+"/orientation", ori);
     pose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(ori[0], ori[1], ori[2]);
  } else
     ROS_WARN("No hay orientacion");

  ros::param::get("~parent_frame", parent_frame_);
  ros::param::get("~frame", frame_);

  int_marker_.header.frame_id = parent_frame_;
  // http://answers.ros.org/question/262866/interactive-marker-attached-to-a-moving-frame/
  // putting a timestamp on the marker makes it not appear
  int_marker_.header.stamp = ros::Time::now();
	int_marker_.name = "/"+ros::this_node::getName()+"/interactive_tf";

  int_marker_.pose = pose_;

  visualization_msgs::InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker_.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker_.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "rotate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker_.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);

	server_->insert(int_marker_);
  // Can't seem to get rid of the 0, _1 parameter
	server_->setCallback(int_marker_.name, boost::bind(&InteractiveTf::processFeedback, this, 0, _1));
	// server_->setCallback(int_marker_.name, testFeedback);
  server_->applyChanges();

  tf_timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&InteractiveTf::updateTf, this, 0, _1));
}

bool InteractiveTf::dumpCB()
{
  if(true || changed_)
  {
    std::vector<double> pos(3);
    std::vector<double> ori(3);

    pos[0] = pose_.position.x;
    pos[1] = pose_.position.y;
    pos[2] = pose_.position.z;
    nh_.setParam("/cameras/"+ros::this_node::getName()+"/position", pos);

    tf::Quaternion q;
    tf::quaternionMsgToTF (pose_.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    ori[0] = roll; ori[1] = pitch; ori[2] = yaw;
    nh_.setParam("/cameras/"+ros::this_node::getName()+"/orientation", ori);

    std::string cmd = "bash -c \"rosparam get /cameras > "+ros::package::getPath("surveillance_system")+"/camera.yaml\"";
    std::cerr<<"GUARDADO!! ["<<cmd<<"]"<<std::endl;
    system(cmd.c_str());
  }
  return true;
}

InteractiveTf::~InteractiveTf()
{
  //establecer los valores de los parámetros a los valores reales, si han cambiado
  dumpCB();
  std::cerr<<"GUARDADO!!"<<std::endl;

  server_.reset();
}


void InteractiveTf::updateTf(int, const ros::TimerEvent& event)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose_.position.x, pose_.position.y, pose_.position.z));
  transform.setRotation(tf::Quaternion(pose_.orientation.x,
      pose_.orientation.y,
      pose_.orientation.z,
      pose_.orientation.w));
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_, frame_));

}

void InteractiveTf::processFeedback(unsigned ind, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  changed_ = true;

  ROS_INFO_STREAM(feedback->header.frame_id);
  pose_ = feedback->pose;
  ROS_DEBUG_STREAM(feedback->control_name);
  ROS_DEBUG_STREAM(feedback->event_type);
  ROS_DEBUG_STREAM(feedback->mouse_point);
	// TODO(lucasw) all the pose changes get handled by the server elsewhere?
	server_->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_tf");
  InteractiveTf interactive_tf;
  ros::spin();
  return 0;
}
