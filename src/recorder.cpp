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

#include <ros/ros.h>
#include <math.h>
#include "std_srvs/Trigger.h"
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>

class Recorder
{
  ros::NodeHandle nh_;
  ros::ServiceServer start_srv;
  ros::ServiceServer stop_srv;
  pid_t pid;

  std::string camera_;
  std::string c;

public:
  Recorder();
  ~Recorder();

  bool start (std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
  bool stop (std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);

};

Recorder::Recorder()
{
  start_srv = nh_.advertiseService("start", &Recorder::start, this);
  stop_srv = nh_.advertiseService("stop", &Recorder::stop, this);

  if (nh_.hasParam("/"+ros::this_node::getName()+"/camera"))
  {
     nh_.getParam("/"+ros::this_node::getName()+"/camera", camera_);
  } else
     ROS_WARN("There isn't camera");


  if(camera_ == "/camera1/rgb/image_raw")
   c = "camera1";

  if(camera_ == "/camera2/rgb/image_raw")
    c = "camera2";
}

Recorder::~Recorder() {}

bool Recorder::start(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
  res.success = true;
  res.message = "ok";
  pid = fork();
  if(pid == 0)
  {
    execlp("rosbag", "rosbag", "record", "--duration=10", "-o", c.c_str(), camera_.c_str(), NULL);
  }
  return true;
}

bool Recorder::stop(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
  res.success = true;
  res.message = "ok";
  int ret = kill(pid,SIGINT);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recorder");
  Recorder recorder;
  ros::spin();
  return 0;
}
