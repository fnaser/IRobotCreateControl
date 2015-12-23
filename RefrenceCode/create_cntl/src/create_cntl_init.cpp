/**
 * \file create_cntl_init.cpp
 *
 * \author Brian J. Julian
 *
 * \version 0.1
 *
 * \date 01 February 2012
 *
 */


// includes
#include <create_cntl.h>


// init params
void CreateCntl::initParams(void)
{
  // get ros parameters
  private_nh_.param("cntl_rate", cntl_rate_, 20.0); //8.0 //4.0
  private_nh_.param("k_x", k_x_, 0.2);
  private_nh_.param("k_yaw", k_yaw_, 10.0);
  private_nh_.param("v_x", v_x_, 0.25);
  private_nh_.param("v_yaw", v_yaw_, 1.94);
  private_nh_.param("S_list", S_list_, S_list_);
}


// init subscribers
void CreateCntl::initSubscribers(void)
{
  if(S_list_.valid())
    {
      ROS_ASSERT(S_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for(int32_t i = 0; i < S_list_.size(); i++)
        {
          ROS_ASSERT(S_list_[i].getType() == XmlRpc::XmlRpcValue::TypeString);
	  string name = string(S_list_[i]);
          S_mp_subs_.push_back(nh_.subscribe<mit_msgs::MocapPosition>(name, 10, boost::bind(&CreateCntl::callbackMocapPosition, this, _1, i)));
          mit_msgs::MocapPosition mp;
          S_mp_.push_back(mp);
	  S_mp_recv_.push_back(false);

          name += "/goal";
          S_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>(name, 10, boost::bind(&CreateCntl::callbackCMsg, this, _1, i)));
          geometry_msgs::PoseStamped ps;
          S_.push_back(ps);
	  S_recv_.push_back(false);
        }
    }
}


// init publishers
void CreateCntl::initPublishers(void)
{
  if(S_list_.valid())
    {
      ROS_ASSERT(S_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for(int32_t i = 0; i < S_list_.size(); i++)
        {
          ROS_ASSERT(S_list_[i].getType() == XmlRpc::XmlRpcValue::TypeString);
          string name = string(S_list_[i]) + "/cntl";
          S_pubs_.push_back(nh_.advertise<std_msgs::Float64MultiArray>(name, 10));
        }
    }
}

// initialize timers
void CreateCntl::initTimers(void)
{
  cntl_timer_ = nh_.createTimer(ros::Duration(1.0/cntl_rate_), &CreateCntl::callbackCntlTimer, this);
}
