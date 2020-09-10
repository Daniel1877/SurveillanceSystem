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

#include "../../../src/surveillance_system/include/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>

#include <iostream>
#include <memory>
#include <vector>

CameraController::CameraController(std::string camera)
{
  start_client = nh_.serviceClient<std_srvs::Trigger>("/"+camera+"/recorders/start");
  stop_client = nh_.serviceClient<std_srvs::Trigger>("/"+camera+"/recorders/stop");
}

bool CameraController::startRecord()
{
  if(start_client.call(srv_))
  {
    return true;
  }
}

bool CameraController::stopRecord()
{
	if(stop_client.call(srv_))
  {
    return true;
  }
}

namespace surveillance_system
{

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , finished_(false)
  , recording1_(false)
  , recording2_(false)
{
  setObjectName("MyPlugin");

  std::shared_ptr<CameraController> c1 = std::shared_ptr<CameraController>(new CameraController("camera1"));
  std::shared_ptr<CameraController> c2 = std::shared_ptr<CameraController>(new CameraController("camera2"));

  camerasControllers.push_back(c1);
  camerasControllers.push_back(c2);
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  connect(ui_.Start, &QPushButton::clicked, this, &MyPlugin::startRec);
  connect(ui_.Stop, &QPushButton::clicked, this, &MyPlugin::stopRec);
  // add widget to the user interface
  context.addWidget(widget_);
}

void MyPlugin::startRec()
{

  if(ui_.Camera1->isChecked())
  {
    if(!recording1_)
    {
      if(camerasControllers[0]->startRecord())
      {
        std::cout << "Start to record from camera 1"<< '\n';
        recording1_ = true;
      }else{
        ROS_WARN("Unable to call start service for camera 1");
      }
    }else{
      ROS_WARN("Camera 1 is already recording");
    }
  }

  if(ui_.Camera2->isChecked())
  {
    if(!recording2_)
    {
      if(camerasControllers[1]->startRecord())
      {
        std::cout << "Start to record from camera 2" << '\n';
        recording2_ = true;
      }else{
        ROS_WARN("Unable to call start service for camera 2");
      }
    }else{
      ROS_WARN("Camera 2 is already recording");
    }
  }
}

void MyPlugin::stopRec()
{
  if(ui_.Camera1->isChecked())
  {
    if(camerasControllers[0]->stopRecord() && recording1_)
    {
        std::cout << "Stop to record camera 1" << '\n';
        recording1_ = false;
    }else if(!camerasControllers[0]->stopRecord()){
      ROS_WARN("Unable to call stop service for camera 1");
    }else{
      ROS_WARN("Camera 1 isn't recording, unable to stop");
    }
  }

  if(ui_.Camera2->isChecked())
  {
    if(camerasControllers[1]->stopRecord() && recording2_)
    {
        std::cout << "Stop to record camera 2" << '\n';
        recording2_ = false;
    }else if(!camerasControllers[0]->stopRecord()){
      ROS_WARN("Unable to call stop service for camera 2");
    }else{
      ROS_WARN("Camera 2 isn't recording, unable to stop");
    }
  }
}

void MyPlugin::shutdownPlugin()
{
  finished_ = true;
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

}  // namespace surveillance_system
PLUGINLIB_DECLARE_CLASS(surveillance_system, MyPlugin, surveillance_system::MyPlugin, rqt_gui_cpp::Plugin)
