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
#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  /*ROS RELATED*/
  this->rosNode.reset(new ros::NodeHandle("contact_plugin"));
  
  // this->initPublisher();
  // // index the topic according to the sensor index
  std::string ROStopicName = "/" + this->parentSensorName + "/contact";
  contact_pub = this->rosNode->advertise<domain_randomization::Contact>(ROStopicName, 10);

  // initialize member variables
  is_contact = false;
  body1_name = "";
  body1_id = 0;
  body2_name = "";
  body2_id = 0;
  sawyer = "sawyer";


  // start thread 
  threadStopSignal = false;
  publishThread = std::thread([=]() {
      // float preMeasuredPressure = 0.0;
      while(!threadStopSignal) {
        // if (fabs(this->measuredPressure- preMeasuredPressure) > 0.0001) {
          publishContact(is_contact, body1_name, body1_id, body2_name, body2_id);
        // }

        // preMeasuredPressure = this->measuredPressure;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
  });

}

void ContactPlugin::publishContact(bool iscontact, std::string body1name, int body1id, std::string body2name, int body2id)
{
  domain_randomization::Contact msg;
  msg.is_contact = iscontact;
  msg.body1_name = body1name;
  msg.body1_id = body1id;
  msg.body2_name = body2name;
  msg.body2_id = body2id;
  contact_pub.publish(msg);
  is_contact = false;
}



/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Uncoment the following lines for debug output
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

      body1_name = contacts.contact(i).collision1();
      body1_id = contacts.contact(i).wrench(i).body_1_id();
      body2_name = contacts.contact(i).collision2();
      body2_id = contacts.contact(i).wrench(i).body_2_id();


      if((body1_name.find(sawyer) != std::string::npos) || (body2_name.find(sawyer) != std::string::npos)) {
        is_contact = true;       
       }
      else
      {
        is_contact = false;
        body1_name = "";
        body1_id = 0;
        body2_name = "";
        body2_id = 0;
      }
    
  }

}




