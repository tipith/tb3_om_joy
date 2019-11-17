/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "tb3_om_joy/tb3_om_joy.h"

#include <map>
#include <string>
#include <cmath>


namespace tb3_om_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TB3OMJoy
 * directly into base nodes.
 */
struct TB3OMJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void jointCallback(const sensor_msgs::JointState::ConstPtr& joy);

  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg);

  ros::Subscriber joy_sub;
  ros::Subscriber joint_sub;

  ros::Publisher om_gripper_position_pub;
  ros::Publisher om_gripper_move_time_pub;

  ros::Publisher om_joint_trajectory_point_pub;
  ros::Publisher om_joint_move_time_pub;

  std::map<std::string, int> axis_map;
  std::map <std::string, int> j2i;
  std::map <std::string, double> llow;
  std::map <std::string, double> lhigh;
  std::map<std::string, double> pos;
  std::map<std::string, double> scaler;

  bool has_received_position;
};



double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

/**
 * Constructs TB3OMJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TB3OMJoy::TB3OMJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->om_joint_trajectory_point_pub = nh->advertise<std_msgs::Float64MultiArray>("joint_trajectory_point", 1, true);
  pimpl_->om_joint_move_time_pub = nh->advertise<std_msgs::Float64>("joint_move_time", 1, true);

  pimpl_->om_gripper_position_pub = nh->advertise<std_msgs::Float64MultiArray>("gripper_position", 1, true);
  pimpl_->om_gripper_move_time_pub = nh->advertise<std_msgs::Float64>("gripper_move_time", 1, true);

  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TB3OMJoy::Impl::joyCallback, pimpl_);
  pimpl_->joint_sub = nh->subscribe<sensor_msgs::JointState>("joint_states", 1, &TB3OMJoy::Impl::jointCallback, pimpl_);

  pimpl_->has_received_position = false;

  pimpl_->pos["joint1"] = 0.0;
  pimpl_->pos["joint2"] = 0.0;
  pimpl_->pos["joint3"] = 0.0;
  pimpl_->pos["joint4"] = 0.0;
  pimpl_->pos["gripper"] = 0.0;

  pimpl_->j2i["joint1"] = 2;
  pimpl_->j2i["joint2"] = 3;
  pimpl_->j2i["joint3"] = 4;
  pimpl_->j2i["joint4"] = 5;
  pimpl_->j2i["gripper"] = 6;

  pimpl_->llow["joint1"] = -deg2rad(100);
  pimpl_->llow["joint2"] = -deg2rad(75);
  pimpl_->llow["joint3"] = -deg2rad(70);
  pimpl_->llow["joint4"] = -deg2rad(80);
  pimpl_->llow["gripper"] = -0.015;  // position+torque controlled, limits from robotis website

  pimpl_->lhigh["joint1"] = -pimpl_->llow["joint1"];
  pimpl_->lhigh["joint2"] = -pimpl_->llow["joint2"];
  pimpl_->lhigh["joint3"] = -pimpl_->llow["joint3"];
  pimpl_->lhigh["joint4"] = -pimpl_->llow["joint4"];
  pimpl_->lhigh["gripper"] = 0.015;

  pimpl_->axis_map["joint1"] = 2;
  pimpl_->axis_map["joint2"] = 3;
  pimpl_->axis_map["joint3"] = -1;
  pimpl_->axis_map["joint4"] = -1;
  pimpl_->axis_map["gripper"] = 4;

  pimpl_->scaler["joint1"] = 1.5;
  pimpl_->scaler["joint2"] = 1;
  pimpl_->scaler["joint3"] = 0;
  pimpl_->scaler["joint4"] = 0;
  pimpl_->scaler["gripper"] = 0.02;
}

double saturate(const double val, const double min, const double max) 
{
    return std::min(std::max(val, min), max);
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg,
              const std::map <std::string, int>& axis,
              const std::map <std::string, double>& scaler,
              const std::map <std::string, double>& pos, 
              const std::string& field)
{
  double diff = joy_msg->axes[axis.at(field)] * scaler.at(field);
  return pos.at(field) + diff;
}

double validate(const double val, const std::map <std::string, double>& low, 
              const std::map <std::string, double>& high, 
              const std::string& field)
{
  double newpos_sat = saturate(val, low.at(field), high.at(field));
  ROS_INFO_NAMED("TB3OMJoy", "field=%s newpos=%f newpos_sat=%f min=%f max=%f", 
                  field.c_str(), val, newpos_sat, low.at(field), high.at(field));
  return newpos_sat;
}

void TB3OMJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  std_msgs::Float64 joint_time_msg;
  joint_time_msg.data = 2;
  om_joint_move_time_pub.publish(joint_time_msg);

  std_msgs::Float64 gripper_time_msg;
  gripper_time_msg.data = 1;
  om_gripper_move_time_pub.publish(gripper_time_msg);

  if (has_received_position)
  {
    std_msgs::Float64MultiArray joint_pos_msg;
    joint_pos_msg.data.push_back(1);  // enable time setting?
    double j1 = getVal(joy_msg, axis_map, scaler, pos, "joint1");
    joint_pos_msg.data.push_back(validate(j1, llow, lhigh, "joint1"));
    double j2 = getVal(joy_msg, axis_map, scaler, pos, "joint2");
    joint_pos_msg.data.push_back(validate(j2, llow, lhigh, "joint2"));
    double j3 = -j2;
    joint_pos_msg.data.push_back(validate(j3, llow, lhigh, "joint3"));
    double j4 = j2 > 0.7 ? 1.5*j2 : 0;
    joint_pos_msg.data.push_back(validate(j4, llow, lhigh, "joint4"));
    om_joint_trajectory_point_pub.publish(joint_pos_msg);

    std_msgs::Float64MultiArray gripper_pos_msg;
    double gripper = getVal(joy_msg, axis_map, scaler, pos, "gripper");
    gripper_pos_msg.data.push_back(validate(gripper, llow, lhigh, "gripper"));
    om_gripper_position_pub.publish(gripper_pos_msg);
  }
}

void TB3OMJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  sendCmdVelMsg(joy_msg);
}

void update_pos(const sensor_msgs::JointState::ConstPtr& joint_msg, 
                const std::string& field, 
                const std::map<std::string, int>& j2i,
                std::map<std::string, double>& posmap)
{
  posmap[field] = joint_msg->position.at(j2i.at(field));
  //ROS_INFO_NAMED("TB3OMJoy", "update_pos: field=%s pos=%f", fieldname.c_str(), posmap.at(field));
}

void TB3OMJoy::Impl::jointCallback(const sensor_msgs::JointState::ConstPtr& joint_msg)
{
  has_received_position = true;
  update_pos(joint_msg, "joint1", j2i, pos);
  update_pos(joint_msg, "joint2", j2i, pos);
  update_pos(joint_msg, "joint3", j2i, pos);
  update_pos(joint_msg, "joint4", j2i, pos);
  update_pos(joint_msg, "gripper", j2i, pos);
}


}  // namespace tb3_om_joy
