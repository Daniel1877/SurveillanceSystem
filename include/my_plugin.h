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

#ifndef SERVICE_PLUGIN_MY_PLUGIN_H
#define SERVICE_PLUGIN_MY_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <../../../build/surveillance_system/ui_my_plugin.h>
//#include <surveillance_system/ui_my_plugin.h>
#include <QWidget>
#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>

class CameraController
{
  ros::NodeHandle nh_;
  ros::ServiceClient start_client;
  ros::ServiceClient stop_client;
  std_srvs::Trigger srv_;

public:
  CameraController(std::string camera);
  bool startRecord();
  bool stopRecord();
};

namespace surveillance_system
{

class MyPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  void startRec();
  void stopRec();

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;

  bool finished_;
  bool recording1_;
  bool recording2_;

  //std::vector<CameraController*> camerasControllers;
  std::vector< std::shared_ptr<CameraController> > camerasControllers;

};
}  // namespace surveillance_system
#endif
