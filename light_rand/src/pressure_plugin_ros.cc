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
#include <boost/algorithm/string.hpp>
#include <gazebo/physics/Base.hh>
#include "pressure_plugin_ros.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(PressurePlugin)

/////////////////////////////////////////////////
PressurePlugin::PressurePlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
PressurePlugin::~PressurePlugin()
{
}

/////////////////////////////////////////////////
void PressurePlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "PressurePlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&PressurePlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // Get world name.
  this->worldName = this->parentSensor->WorldName();

  // Get name of parent sensor.
  this->parentSensorName = this->parentSensor->Name();

  // Get collision names of parent sensor and physics pointers.
  physics::WorldPtr world = physics::get_world(this->worldName);
  // Initialize ROS related stuff
  this->rosNode.reset(new ros::NodeHandle("pressure_plugin"));
  
  // this->initPublisher();
  // // index the topic according to the sensor index
  std::string ROStopicName = "/" + this->parentSensorName + "/tactile";
  tactile_pressure_pub = this->rosNode->advertise<std_msgs::Float32>(ROStopicName, 10);


  unsigned int collisionCount = this->parentSensor->GetCollisionCount();
  for (unsigned int i = 0; i < collisionCount; ++i)
  {
    std::string collisionScopedName = this->parentSensor->GetCollisionName(i);

    ROS_WARN("CollisionScopedName == %s", collisionScopedName);

    // Strip off ::collision_name to get link name
    std::string linkName = collisionScopedName.substr(0,
                           collisionScopedName.rfind("::"));
    ROS_WARN("LinkName == %s", linkName);
    // Get unscoped name of collision
    std::string collisionName =
      collisionScopedName.substr(collisionScopedName.rfind("::") + 2);

    ROS_WARN("CollisionName == %s", linkName);

    // Get physics pointers
    physics::EntityPtr entity = world->GetEntity(linkName);
    if (entity && entity->HasType(physics::Base::LINK))
    {
      physics::LinkPtr link =
        boost::dynamic_pointer_cast<physics::Link>(entity);
      if (link)
      {
        physics::CollisionPtr collision = link->GetCollision(collisionName);
        if (collision)
        {
          physics::ShapePtr shape = collision->GetShape();
          if (shape->HasType(physics::Base::BOX_SHAPE))
          {
            physics::BoxShapePtr box =
              boost::dynamic_pointer_cast<physics::BoxShape>(shape);
            if (box)
            {
              math::Vector3 size = box->GetSize();
              std::vector<double> sizeVector;
              sizeVector.push_back(size.x);
              sizeVector.push_back(size.y);
              sizeVector.push_back(size.z);
              std::sort(sizeVector.begin(), sizeVector.end());
              double area = sizeVector[1] * sizeVector[2];
              if (area > 0.0)
                this->collisionNamesToArea[collisionScopedName] = area;
            }
          }
        }
      }
    }
  }
}

// void PressurePlugin::initPublisher()
// {
//   // index the topic according to the sensor index
//     ros::Publisher pressure_Publisher;
//     std::string ROStopicName = "/" + this->parentSensorName + "/tactile";
//     pressure_Publisher = this->rosNode->advertise<std_msgs::Float32>(ROStopicName, 10);

// }
void PressurePlugin::publishPressure(float measured_pressure)
{
  std_msgs::Float32 msg;
  msg.data = measured_pressure;
  tactile_pressure_pub.publish(msg);
}



/////////////////////////////////////////////////
void PressurePlugin::Init()
{
  this->node.reset(new transport::Node());
  this->node->Init(this->worldName);

  if (!this->parentSensorName.empty())
  {
    // Create publisher for tactile messages
    std::string topicName = "~/" + this->parentSensorName + "/tactile";
    boost::replace_all(topicName, "::", "/");
    this->tactilePub = this->node->Advertise<msgs::Tactile>(topicName);
  }
}

/////////////////////////////////////////////////
void PressurePlugin::OnUpdate()
{
  msgs::Tactile tactileMsg;

  // For each collision attached to this sensor
  boost::unordered_map<std::string, double>::iterator iter;
  for (iter = this->collisionNamesToArea.begin();
       iter != this->collisionNamesToArea.end(); ++iter)
  {
    double normalForceSum = 0, normalForce;
    // Get the contacts sorted by collision element.
    std::map<std::string, gazebo::physics::Contact> contacts;
    std::map<std::string, gazebo::physics::Contact>::iterator iter2;
    contacts = this->parentSensor->Contacts(iter->first);

    for (unsigned int i = 0; i < contacts.contact_size(); ++i)
      {
        std::cout << "Collision between[" << contacts.contact(i).collision1()
                  << "] and [" << contacts.contact(i).collision2() << "]\n";

        for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
        {
          std::cout << j << "  Position:"
                    << contacts.contact(i).position(j).x() << " "
                    << contacts.contact(i).position(j).y() << " "
                    << contacts.contact(i).position(j).z() << "\n";
          std::cout << "   Normal:"
                    << contacts.contact(i).normal(j).x() << " "
                    << contacts.contact(i).normal(j).y() << " "
                    << contacts.contact(i).normal(j).z() << "\n";
          std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
        }
      }

    for (iter2 = contacts.begin(); iter2 != contacts.end(); ++iter2)
    {
      for (int i = 0; i < iter2->second.count; ++i)
      {
        // TODO: determine whether body1Force or body2Force should be used.
        normalForce = iter2->second.normals[i].x *
                      iter2->second.wrench[i].body1Force.x +
                      iter2->second.normals[i].y *
                      iter2->second.wrench[i].body1Force.y +
                      iter2->second.normals[i].z *
                      iter2->second.wrench[i].body1Force.z;
        normalForceSum += normalForce;
      }
    }
    ROS_WARN("Normal force sum == %f", normalForceSum);

    if (normalForceSum > 0)
    {
      tactileMsg.add_collision_name(iter->first);
      tactileMsg.add_collision_id(0);
      tactileMsg.add_pressure(normalForceSum / iter->second);
      // Publish via ROS nodes
      this->publishPressure(normalForceSum / iter->second);
    }
  }

  msgs::Contacts contacts = this->parentSensor->Contacts();
  int nc = contacts.contact_size();
  if (nc > 0)
  {
    common::Time currentContactTime;
    currentContactTime = msgs::Convert(contacts.contact(nc-1).time());
    msgs::Set(tactileMsg.mutable_time(), currentContactTime);

    if (this->tactilePub && tactileMsg.pressure_size() > 0)
      this->tactilePub->Publish(tactileMsg);
  }
}
