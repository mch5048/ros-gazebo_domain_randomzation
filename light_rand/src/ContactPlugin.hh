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
 * Desc: Contact Plugin
 * Author: Nate Koenig mod by John Hsu
 */
#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/util/system.hh"
#include <domain_randomization/Contact.h>

// ROS
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/advertise_options.h"
#include "std_msgs/Float32.h"
#include <std_srvs/Empty.h>

// Required fields workaround
#include <chrono>
#include <thread>
#include <atomic>


namespace gazebo
{
  /// \brief A plugin for a contact sensor. Inherit from this class to make
  /// your own contact plugin.
  class GAZEBO_VISIBLE ContactPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that recieves the contact sensor's update signal.
    /// Override this this function to get callbacks when the contact sensor
    /// is updated with new data.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    private: std::string parentSensorName;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

 // Define ROS service nodes 
    // \ brief a pointer to the ROS node. A node will be instantiated if it does not exist.
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::ServiceServer srv_;
    private: ros::CallbackQueue callback_queue_;
    private: boost::thread callback_queue_thread_;
    private: std::string serviceName;

    // Define publisher -> Be aware of indexing the topic name according to the sensor indices.
    ros::Publisher contact_pub;

    // Thread parameter
    pthread_t timer_thread_;
    pthread_attr_t attr_;

    // message contents
    bool is_contact;
    std::string body1_name;
    int body1_id;
    std::string body2_name;
    int body2_id;


    std::string sawyer;

    void publishCallback(const ros::TimerEvent&);
    void initPublisher();
    void publishContact(bool iscontact, std::string body1name, int body1id, std::string body2name, int body2id);

    private:
      std::atomic<bool> threadStopSignal;
      std::thread publishThread;
  };
}
#endif
