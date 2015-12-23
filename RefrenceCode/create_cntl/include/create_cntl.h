/**
 * \file create_cntl.h
 *
 * \author Brian J. Julian
 *
 * \version 0.2
 *
 * \date 01 February 2012
 *
 */


#ifndef __CREATE_CNTL_H__
#define __CREATE_CNTL_H__


// includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <mit_msgs/MocapPosition.h>
#include <geometry_msgs/PoseStamped.h>


// namespace
using namespace std;


#ifndef ABS
#define ABS(A) (((A)<(0))?(-A):(A))
#endif

#ifndef SIGN
#define SIGN(A) (((A)<(0))?('-'):(((A)>(0))?('+'):('0')))
#endif

#ifndef MIN
#define MIN(A,B) (((A)<(B))?(A):(B))
#endif

#ifndef MAX
#define MAX(A,B) (((A)>(B))?(A):(B))
#endif

#ifndef MINMAX
#define MINMAX(A,B,C) MIN((B),MAX((A),(C)))
#endif


// macros
#ifndef NORMALIZE
#define NORMALIZE(Z) atan2(sin(Z),cos(Z))
#endif


// class
class CreateCntl
{
public:
 
  // constructor
  CreateCntl();

  // destructor
  ~CreateCntl();

private:

  // init
  void initParams(void);
  void initSubscribers(void);
  void initPublishers(void);
  void initTimers(void);

  // callback
  void callbackCMsg(const geometry_msgs::PoseStamped::ConstPtr &msg, const int32_t &index);
  void callbackMocapPosition(const mit_msgs::MocapPosition::ConstPtr &msg, const int32_t &index);
  void callbackCntlTimer(const ros::TimerEvent &event);

  // ros objects
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros subscribers
  ros::V_Subscriber S_subs_;
  ros::V_Subscriber S_mp_subs_;

  // ros publishers
  ros::V_Publisher S_pubs_;

  // ros timers
  ros::Timer cntl_timer_;

  // ros param
  double cntl_rate_;
  double k_x_;
  double k_yaw_;
  double v_x_;
  double v_yaw_;
  XmlRpc::XmlRpcValue S_list_;

  // other containers
  vector<geometry_msgs::PoseStamped> S_;
  vector<mit_msgs::MocapPosition> S_mp_;
  vector<bool> S_recv_;
  vector<bool> S_mp_recv_;
};


#endif // __CREATE_CNTL_H__
