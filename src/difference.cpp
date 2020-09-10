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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "std_srvs/Trigger.h"
#include <cstdlib>
#include <string>

#include <ros/time.h>

class Difference
{
  ros::NodeHandle nh_;
  pcl::PointCloud<pcl::PointXYZ>  last_data;
  pcl::PointCloud<pcl::PointXYZ>  data;
  bool started_;

  ros::Subscriber sub;
  ros::Publisher pub;

  ros::ServiceClient start_client;
  ros::ServiceClient stop_client;
  std_srvs::Trigger srv_;

  bool recording_;
  ros::Time start_ts_;

  std::string grabar;
  std::string parar;

public:
  Difference():
    started_(false),
    recording_(false)
    {
      //Create a ROS subscriber for the input point cloud
      sub = nh_.subscribe ("input", 1, &Difference::cloud_cb, this);
      //Create a ROS publisher for the output model coefficients
      pub = nh_.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

      if (nh_.hasParam("/"+ros::this_node::getName()+"/start"))
      {
         nh_.getParam("/"+ros::this_node::getName()+"/start", grabar);
      } else
         ROS_WARN("No start service found");

     if (nh_.hasParam("/"+ros::this_node::getName()+"/stop"))
     {
        nh_.getParam("/"+ros::this_node::getName()+"/stop", parar);
     } else
        ROS_WARN("No stop service found");

      start_client = nh_.serviceClient<std_srvs::Trigger>(grabar);
      stop_client = nh_.serviceClient<std_srvs::Trigger>(parar);

    }

    ~Difference(){}

    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
    {
      // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
      pcl::fromROSMsg (*input, data);
      if(!started_) {
        last_data = data;
        started_ = true;
      }
    }

    void step()
    {
      if(!started_) return;

      if (recording_)
      {
        if((ros::Time::now() - start_ts_).toSec() > 10)
        {
          recording_ = false;
          std::cout << "STOP" << '\n';
        }
      }
      else
      {
        double diff_x=0.0, diff_y=0.0, diff_z=0.0;
        long counter = 0;
        size_t j = 0;
        while(j < data.size() && counter < 5100)
        {
          diff_x = std::abs(data.points[j].x - last_data.points[j].x);
          diff_y = std::abs(data.points[j].y - last_data.points[j].y);
          diff_z = std::abs(data.points[j].z - last_data.points[j].z);

          double dist = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);

          if(dist>0.5) counter++;
          j++;
        }
        if(counter > 5000)
        {
          if (start_client.call(srv_))
          {
            start_ts_ = ros::Time::now();
            std::cout << "START" << '\n';
            recording_ = true;
          }else
          {
            ROS_WARN("Unable to start service");
          }
        }
        last_data = data;
      }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "difference");
  Difference difference;
  ros::Rate rate(1);
  while(ros::ok())
  {
    difference.step();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
