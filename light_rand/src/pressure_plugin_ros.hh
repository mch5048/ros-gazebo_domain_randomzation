/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
 *
*/
/*
 * Desc: Pressure Sensor Plugin with ROS Service
 * Author: Cheolhui Min
 */
#ifndef _GAZEBO_PRESSURE_PLUGIN_HH_
#define _GAZEBO_PRESSURE_PLUGIN_HH_

#include <string>
#include <boost/unordered_map.hpp>

// ROS
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/advertise_options.h"
#include "std_msgs/Float32.h"
#include <std_srvs/Empty.h>




#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include "gazebo/util/system.hh"

// Required fields workaround
#include <limits>

// Required fields workaround
#include <chrono>
#include <thread>

namespace gazebo
{
  /// \brief A plugin for a tactile pressure sensor.
  /// The plugin extends the contact sensor and is currently limited to
  /// collision elements with BoxShape geometry. It is assumed that pressure
  /// measurement occurs on the largest face of the box. All normal forces on
  /// each box shape are summed and divided by the area of the largest face
  /// to compute pressure.
  class GAZEBO_VISIBLE PressurePlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: PressurePlugin();

    /// \brief Destructor.
    public: virtual ~PressurePlugin();

    // Documentation inherited.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Callback that recieves the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    /// \brief Transport node used for publishing tactile messages.
    private: transport::NodePtr node;

    /// \brief Publisher of tactile messages.
    private: transport::PublisherPtr tactilePub;

    /// \brief World name.
    private: std::string worldName;

    /// \brief Parent sensor name.
    private: std::string parentSensorName;

    /// \brief Parent sensor collision names.
    private: boost::unordered_map<std::string, double> collisionNamesToArea;

    // Define ROS service nodes 
    // \ brief a pointer to the ROS node. A node will be instantiated if it does not exist.
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::ServiceServer srv_;
    private: ros::CallbackQueue callback_queue_;
    private: boost::thread callback_queue_thread_;
    private: std::string serviceName;

    // Define publisher -> Be aware of indexing the topic name according to the sensor indices.
    ros::Publisher tactile_pressure_pub;

    // Thread parameter
    pthread_t timer_thread_;
    pthread_attr_t attr_;

    void publishCallback(const ros::TimerEvent&);
    void initPublisher();
    void publishPressure(float measured_pressure);

  };
}
#endif
