/**
 * \file create_cntl.cpp
 *
 * \author Brian J. Julian
 *
 * \version 0.1
 *
 * \date 01 February 2012
 *
 */


// defines
#define ROS_PKG "create_cntl"
#define ROS_NODE "create_cntl"


// includes
#include <create_cntl.h>


// class constructor
CreateCntl::CreateCntl(void)
  : nh_(),
    private_nh_("~")
{
  // init parameters
  initParams();

  // init subscribers
  initPublishers();

  // init subscribers
  initSubscribers();

  // init subscribers
  initTimers();
}


// class destructor
CreateCntl::~CreateCntl(void)
{ }


// main function
int main(int argc, char **argv)
{
  // initialize ros node handle
  ros::init(argc, argv, ROS_NODE);

  // print start to log file
  ROS_INFO("Started node %s", ROS_NODE);

  // create class instance
  CreateCntl create_cntl;

  // ros spin
  ros::spin();

  // print termination to log file
  ROS_INFO("Stopped node %s", ROS_NODE);

  // return success
  return(0);           
}
