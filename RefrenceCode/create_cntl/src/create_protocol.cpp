/**
 * \file create_protocol.cpp
 *
 * \author Brian J. Julian
 *
 * \version 0.1
 *
 * \date 08 October 2010
 *
 */

// includes
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>


// defines
#define ROS_PKG "create_protocol"
#define ROS_NODE "create_protocol"

#ifndef MIN
#define MIN(A,B) (((A)<(B))?(A):(B))
#endif

#ifndef MAX
#define MAX(A,B) (((A)>(B))?(A):(B))
#endif

#ifndef MINMAX
#define MINMAX(A,B,C) MIN( (B),MAX((A),(C)) )
#endif

using namespace std;


// class
class CreateProtocol
{
public:
 
  // constructor
  CreateProtocol();

  // destructor
  ~CreateProtocol();

private:

  // subscriber callback
  void callbackCntlInputMsg(const boost::shared_ptr<std_msgs::Float64MultiArray const> &cntl_input_msg, const int32_t &i);
  void callbackCntlTimer(const ros::TimerEvent &event);

  // ros objects
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros timers
  ros::Timer cntl_timer_;

  // ros param
  XmlRpc::XmlRpcValue S_list_;
  double cntl_rate_;

  // servo topic objects
  ros::V_Subscriber cntl_input_subs_;
  ros::V_Publisher serial_command_pubs_;
  vector<std_msgs::Float64MultiArray> cntl_inputs_;
  volatile int32_t counter_;
};


// class constructor
CreateProtocol::CreateProtocol(void)
  : nh_(),
    private_nh_("~")
{
  // local stack
  private_nh_.param("S_list", S_list_, S_list_);
  private_nh_.param("cntl_rate", cntl_rate_, 10.0);//2.0, 10.0 // Was originally 5.0 : Swarun
  counter_ = 0;

  // parse list
  if(S_list_.valid())
    {
      ROS_ASSERT(S_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for(int32_t i = 0; i < S_list_.size(); i++)
        {
          ROS_ASSERT(S_list_[i].getType() == XmlRpc::XmlRpcValue::TypeString);
	  string name = string(S_list_[i]) + "/cntl";
	  cntl_input_subs_.push_back(nh_.subscribe<std_msgs::Float64MultiArray>(name, 10, boost::bind(&CreateProtocol::callbackCntlInputMsg, this, _1, i)));
	  name = string(S_list_[i]) + "/serial_command";
	  serial_command_pubs_.push_back(nh_.advertise<std_msgs::UInt8MultiArray>(name, 1));
	  std_msgs::Float64MultiArray float64array;
	  float64array.data.push_back(0);
	  float64array.data.push_back(0);
	  cntl_inputs_.push_back(float64array);
	}

      if(S_list_.size() > 0)
	{
	  // init timers
	  cntl_timer_ = nh_.createTimer(ros::Duration(1.0/(double(S_list_.size())*cntl_rate_)), &CreateProtocol::callbackCntlTimer, this);
	}
    }
}


// class destructor
CreateProtocol::~CreateProtocol(void)
{
  // stop all threads
}


void CreateProtocol::callbackCntlTimer(const ros::TimerEvent &event)
{
  unsigned char c[8];
  unsigned char c_temp[2];
  int16_t int16;
  double float64;
  std_msgs::UInt8MultiArray serial_command;
  if(cntl_inputs_[counter_].data.size() == 2)
    {
      c[0] = 128;
      c[1] = 132;
      c[2] = 145;

      // Swarun: START      
      double factor = 1.0; //1.2; //2.0; moves faster if factor>1.0
      int sign_right = (cntl_inputs_[counter_].data[0] >= 0);
      int sign_left  = (cntl_inputs_[counter_].data[1] >= 0);
      if(sign_right == sign_left)  { //move slower along straight line trajectories (factor small means slower)
	factor = 3.0; //0.8//1.0 
      }
      // This makes clients: create10, create5, create6 (index 2, 3, 4) move fast if factor>1.0
      if(counter_ == 2 || counter_ == 3 || counter_ == 4) {
	//factor = 1.0;//3.0
      }      
      // Swarun: END

      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[0]*1.5); // Fast
      // Swarun: START      
      //if(counter_ == 4) { // 4: is the index for create6
      // float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[0]*3); // Fast
      //} else {      
      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[0]); // Original
      //}
      // Swarun: END

      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[0]); // Original
      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[0]*0.5); // Slow
      float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[0]*factor); // Swarun
      
      int16 = short(float64);
      memcpy(&c_temp, (unsigned char *)&int16, sizeof(short));
      c[3] = c_temp[1];
      c[4] = c_temp[0];

      // Swarun: START
      //if(counter_ == 4) { // 4: is the index for create6
      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[1]*3); // Fast
      //} else {      
      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[1]); // Original
      //}
      // Swarun: END
      
      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[1]); // Original
      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[1]*1.5); // Fast
      //float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[1]*0.5); // Slow
      float64 = MINMAX(-499,499,cntl_inputs_[counter_].data[1]*factor); // Swarun

      int16 = short(float64);
      memcpy(&c_temp, (unsigned char *)&int16, sizeof(short));
      c[5] = c_temp[1];
      c[6] = c_temp[0];
      serial_command.data.reserve(7);
      serial_command.data.assign(c, c+7);
      // publish serial command
      serial_command_pubs_[counter_].publish(serial_command);
    }
  counter_ = (counter_+1)%(S_list_.size());
}



// callback for servo command message
void CreateProtocol::callbackCntlInputMsg(const boost::shared_ptr<std_msgs::Float64MultiArray const> &cntl_input_msg, const int32_t &i)
{
  cntl_inputs_[i] = *cntl_input_msg;
}


// main function
int main(int argc, char **argv)
{
  // initialize ros node handle
  ros::init(argc, argv, ROS_NODE);

  // print start to log file
  ROS_INFO("Started node %s", ROS_NODE);

  // create class instance
  CreateProtocol create_protocol;

  // ros spin
  ros::spin();

  // print termination to log file
  ROS_INFO("Stopped node %s", ROS_NODE);

  // return success
  return(0);           
}
