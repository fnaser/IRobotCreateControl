/**
 * \file matlab_protocol.h
 *
 * \author Brian J. Julian
 *
 * \version 0.2
 *
 * \date 01 February 2012
 *
 */


#ifndef __MATLAB_PROTOCOL_H__
#define __MATLAB_PROTOCOL_H__


// includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <mit_msgs/MocapPosition.h>
#include <geometry_msgs/PoseStamped.h>


// namespace
using namespace std;


// macros
#ifndef NORMALIZE
#define NORMALIZE(Z) atan2(sin(Z),cos(Z))
#endif


// class
class MatlabProtocol
{
public:
 
  // constructor
  MatlabProtocol();

  // destructor
  ~MatlabProtocol();

private:

  // init
  void initParams(void);
  void initSubscribers(void);
  void initPublishers(void);

  // callback
  void callbackMatlabMsg(const std_msgs::String::ConstPtr &str_msg);
  void callbackCMsg(const mit_msgs::MocapPosition::ConstPtr &msg, const int32_t &index);
  void callbackSMsg(const mit_msgs::MocapPosition::ConstPtr &msg, const int32_t &index);
  void callbackTMsg(const std_msgs::Float64MultiArray::ConstPtr &msg);

  // ros objects
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros subscribers
  ros::Subscriber matlab_sub_;
  ros::V_Subscriber C_subs_;
  ros::V_Subscriber S_subs_;
  ros::Subscriber T_sub_;

  // ros publishers
  ros::Publisher matlab_pub_;
  ros::V_Publisher C_pubs_;
  ros::V_Publisher S_pubs_;
  ros::Publisher S_pub_;
  ros::Publisher T_pub_;

  // ros param
  double x_scale_;
  double y_scale_;
  double z_height_;
  XmlRpc::XmlRpcValue C_list_;
  XmlRpc::XmlRpcValue S_list_;

  // other containers
  vector<mit_msgs::MocapPosition> C_;
  vector<mit_msgs::MocapPosition> S_;
  std_msgs::Float64MultiArray T_;
};


#endif // __MATLAB_PROTOCOL_H__
