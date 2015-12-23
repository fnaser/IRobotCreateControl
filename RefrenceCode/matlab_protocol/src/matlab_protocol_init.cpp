/**
 * \file matlab_protocol_init.cpp
 *
 * \author Brian J. Julian
 *
 * \version 0.1
 *
 * \date 01 February 2012
 *
 */


// includes
#include <matlab_protocol.h>


// init params
void MatlabProtocol::initParams(void)
{
  // get ros parameters
  private_nh_.param("x_scale", x_scale_, 1.0);
  private_nh_.param("y_scale", y_scale_, 1.0);
  private_nh_.param("z_height", z_height_, 100.0);
  private_nh_.param("C_list", C_list_, C_list_);
  private_nh_.param("S_list", S_list_, S_list_);
}


// init subscribers
void MatlabProtocol::initSubscribers(void)
{
  if(C_list_.valid())
    {
      ROS_ASSERT(C_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for(int32_t i = 0; i < C_list_.size(); i++)
        {
          ROS_ASSERT(C_list_[i].getType() == XmlRpc::XmlRpcValue::TypeString);
          string name = string(C_list_[i]);
          C_subs_.push_back(nh_.subscribe<mit_msgs::MocapPosition>(name, 10, boost::bind(&MatlabProtocol::callbackCMsg, this, _1, i)));
          mit_msgs::MocapPosition mp;
          C_.push_back(mp);
        }
    }

  if(S_list_.valid())
    {
      ROS_ASSERT(S_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for(int32_t i = 0; i < S_list_.size(); i++)
        {
          ROS_ASSERT(S_list_[i].getType() == XmlRpc::XmlRpcValue::TypeString);
          string name = string(S_list_[i]);
          S_subs_.push_back(nh_.subscribe<mit_msgs::MocapPosition>(name, 10, boost::bind(&MatlabProtocol::callbackSMsg, this, _1, i)));
          mit_msgs::MocapPosition mp;
          S_.push_back(mp);
        }
    }
  
  T_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("T", 10, &MatlabProtocol::callbackTMsg, this);
  matlab_sub_ = nh_.subscribe<std_msgs::String>("matlab_sub", 10, &MatlabProtocol::callbackMatlabMsg, this);
}


// init publishers
void MatlabProtocol::initPublishers(void)
{
  if(C_list_.valid())
    {
      ROS_ASSERT(C_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for(int32_t i = 0; i < C_list_.size(); i++)
        {
          ROS_ASSERT(C_list_[i].getType() == XmlRpc::XmlRpcValue::TypeString);
          string name = string(C_list_[i]) + "/goal";
          C_pubs_.push_back(nh_.advertise<geometry_msgs::PoseStamped>(name, 10));
        }
    }

  if(S_list_.valid())
    {
      ROS_ASSERT(S_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for(int32_t i = 0; i < S_list_.size(); i++)
        {
          ROS_ASSERT(S_list_[i].getType() == XmlRpc::XmlRpcValue::TypeString);
          string name = string(S_list_[i]) + "/goal";
          S_pubs_.push_back(nh_.advertise<geometry_msgs::PoseStamped>(name, 10));
        }
    }

  T_pub_ = nh_.advertise<std_msgs::Empty>("T_start", 10);
  S_pub_ = nh_.advertise<std_msgs::Empty>("S_start", 10);
  matlab_pub_ = nh_.advertise<std_msgs::String>("matlab_pub", 10);
}
