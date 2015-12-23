/**
 * \file matlab_protocol.cpp
 *
 * \author Stephanie Gil and Brian J. Julian
 *
 * \version 0.1
 *
 * \date 07 June 2013
 *
 */


// defines
#define ROS_PKG "matlab_protocol"
#define ROS_NODE "matlab_protocol"


// includes
#include <matlab_protocol_sgil.h>


// class constructor
MatlabProtocolGil::MatlabProtocolGil(void)
  : nh_(),
    private_nh_("~")
{
  // init parameters
  initParams();

  // init subscribers
  initPublishers();

  // init subscribers
  initSubscribers();
}


// class destructor
MatlabProtocolGil::~MatlabProtocolGil(void)
{ }


// main function
int main(int argc, char **argv)
{
  // initialize ros node handle
  ros::init(argc, argv, ROS_NODE);

  // print start to log file
  ROS_INFO("Started node %s", ROS_NODE);

  // create class instance
  MatlabProtocolGil matlab_protocol_sgil;

  // ros spin
  ros::spin();

  // print termination to log file
  ROS_INFO("Stopped node %s", ROS_NODE);

  // return success
  return(0);           
}
