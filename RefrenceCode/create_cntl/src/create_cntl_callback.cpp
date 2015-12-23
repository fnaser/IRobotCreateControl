/**
 * \file create_cntl_callback.cpp
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


void CreateCntl::callbackCntlTimer(const ros::TimerEvent &event)
{
  for(uint32_t i = 0; i < S_.size(); i++)
    {
      if(S_recv_[i] && S_mp_recv_[i])
	{
	  double cntl_x = 0.0;
	  double cntl_y = 0.0;
	  double cntl_yaw = 0.0;
	  double cntl_b_x = 0.0;

	  double d_x = S_[i].pose.position.x - S_mp_[i].translational.x/1000.0;
	  double d_y = S_[i].pose.position.y - S_mp_[i].translational.y/1000.0;
	  double d_mag = sqrt(d_x*d_x + d_y*d_y);

	  if(fabs(d_mag) > 0.10)//0.05
	    {
	      double yaw = S_mp_[i].axisangle.z;

	      double b_x = cos(yaw)*d_x + sin(yaw)*d_y;
	      double b_y = cos(yaw)*d_y - sin(yaw)*d_x;
	      double b_yaw = atan2(b_y,b_x);

	      cntl_b_x = k_x_*b_x;
	      if(fabs(cntl_b_x) > v_x_)
		{
		  cntl_b_x *= v_x_/fabs(cntl_b_x);
		}
	      cntl_x = cos(yaw)*cntl_b_x;
	      cntl_y = sin(yaw)*cntl_b_x;

	      cntl_yaw = k_yaw_*b_yaw;
	      if(fabs(cntl_yaw) > v_yaw_)
		{
		  cntl_yaw *= v_yaw_/fabs(cntl_yaw);
		}
	    }

	  std_msgs::Float64MultiArray float64array;
	  cntl_b_x = MAX(cntl_b_x,0.0);

	  float64array.data.push_back(100.0*cntl_b_x/v_x_ + 75.0*cntl_yaw/v_yaw_);
	  float64array.data.push_back(100.0*cntl_b_x/v_x_ - 75.0*cntl_yaw/v_yaw_);
	  S_pubs_[i].publish(float64array);
	}
    }
}


void CreateCntl::callbackCMsg(const geometry_msgs::PoseStamped::ConstPtr &msg, const int32_t &index)
{
  S_[index] = *msg;
  S_recv_[index] = true;
}


void CreateCntl::callbackMocapPosition(const mit_msgs::MocapPosition::ConstPtr &msg, const int32_t &index)
{
  S_mp_[index] = *msg;
  S_mp_recv_[index] = true;
}
