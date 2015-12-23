/**
 * \file matlab_protocol_callback_sgil.cpp
 *
 * \author Stephanie Gil and Brian J. Julian
 *
 * \version 0.1
 *
 * \date 07 June 2013
 *
 */


// includes
#include <matlab_protocol_sgil.h>


void MatlabProtocolGil::callbackMatlabMsg(const std_msgs::String::ConstPtr &str_msg)
{
  if(str_msg->data.size() == 1)
    {
      if(str_msg->data == "C")
        {
          std_msgs::String msg;
          double float64;
          for(uint32_t i = 0; i < C_.size(); i++)
            {
              float64 = C_[i].translational.x/x_scale_;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = C_[i].translational.y/y_scale_;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = C_[i].translational.z;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = C_[i].axisangle.x;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = C_[i].axisangle.y;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = C_[i].axisangle.z;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
            }
          matlab_pub_.publish(msg);
        }
      else if(str_msg->data == "S")
        {
          std_msgs::String msg;
          double float64;
          for(uint32_t i = 0; i < S_.size(); i++)
            {
              float64 = S_[i].translational.x/x_scale_;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = S_[i].translational.y/y_scale_;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = S_[i].translational.z;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = S_[i].axisangle.x;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = S_[i].axisangle.y;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
              float64 = S_[i].axisangle.z;
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
            }
          matlab_pub_.publish(msg);
        }
      else if(str_msg->data == "T")
        {
          std_msgs::String msg;
          double float64;
          for(uint32_t i = 0; i < T_.data.size(); i++)
            {
              float64 = T_.data[i];
              msg.data.append(reinterpret_cast<uint8_t *>(&float64), reinterpret_cast<uint8_t *>(&float64)+sizeof(float64));
            }
          matlab_pub_.publish(msg);
        }
      else if(str_msg->data == "t")
        {
          std_msgs::Empty msg;
          T_pub_.publish(msg);
        }
      else if(str_msg->data == "s")
        {
          std_msgs::Empty msg;
          S_pub_.publish(msg);
        }
    }
  else
    {
      if(str_msg->data.size() == ((6*C_.size()+1)*sizeof(double)))
        {
          string msg = str_msg->data;
          string::iterator it = msg.begin();
          double float64;
          copy(it, it+sizeof(float64), reinterpret_cast<char *>(&float64));
          it += sizeof(float64);

          if((float64 > 9.999999) && (float64 < 10.000001))
            {
              double roll, pitch, yaw;
              for(uint32_t i = 0; i < C_.size(); i++)
                {
                  geometry_msgs::PoseStamped pose_msg;
                  pose_msg.header.stamp = ros::Time::now();

                  copy(it, it+sizeof(float64), reinterpret_cast<char *>(&float64));
                  pose_msg.pose.position.x = float64*x_scale_;
                  it += sizeof(float64);

                  copy(it, it+sizeof(float64), reinterpret_cast<char *>(&float64));
                  pose_msg.pose.position.y = float64*y_scale_;
                  it += sizeof(float64);

                  copy(it, it+sizeof(float64), reinterpret_cast<char *>(&float64));
                  //pose_msg.pose.position.z = float64;
		  pose_msg.pose.position.z = z_height_+double(i)*500.0;
                  it += sizeof(float64);

                  copy(it, it+sizeof(roll), reinterpret_cast<char *>(&roll));
                  it += sizeof(roll);

                  copy(it, it+sizeof(pitch), reinterpret_cast<char *>(&pitch));
                  it += sizeof(pitch);

                  copy(it, it+sizeof(yaw), reinterpret_cast<char *>(&yaw));
                  it += sizeof(yaw);

                  //pose_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
		  pose_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

                  C_pubs_[i].publish(pose_msg);
                }
              std_msgs::String msg_out;
              msg_out.data = "c";
              matlab_pub_.publish(msg_out);
            }
        }

      if(str_msg->data.size() == ((6*S_.size()+1)*sizeof(double)))
        {
          string msg = str_msg->data;
          string::iterator it = msg.begin();
          double float64;
          copy(it, it+sizeof(float64), reinterpret_cast<char *>(&float64));
          it += sizeof(float64);

          if((float64 > 13.999999) && (float64 < 14.000001))
            {
              double roll, pitch, yaw;
              for(uint32_t i = 0; i < S_.size(); i++)
                {
                  geometry_msgs::PoseStamped pose_msg;
                  pose_msg.header.stamp = ros::Time::now();

                  copy(it, it+sizeof(float64), reinterpret_cast<char *>(&float64));
                  pose_msg.pose.position.x = float64*x_scale_;
                  it += sizeof(float64);

                  copy(it, it+sizeof(float64), reinterpret_cast<char *>(&float64));
                  pose_msg.pose.position.y = float64*y_scale_;
                  it += sizeof(float64);

                  copy(it, it+sizeof(float64), reinterpret_cast<char *>(&float64));
                  //pose_msg.pose.position.z = float64;
		  pose_msg.pose.position.z = z_height_+double(i)*500.0;
                  it += sizeof(float64);

                  copy(it, it+sizeof(roll), reinterpret_cast<char *>(&roll));
                  it += sizeof(roll);

                  copy(it, it+sizeof(pitch), reinterpret_cast<char *>(&pitch));
                  it += sizeof(pitch);

                  copy(it, it+sizeof(yaw), reinterpret_cast<char *>(&yaw));
                  it += sizeof(yaw);

                  //pose_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
		  pose_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

                  S_pubs_[i].publish(pose_msg);
                }
              std_msgs::String msg_out;
              msg_out.data = "s";
              matlab_pub_.publish(msg_out);
            }
        }
    }
}


void MatlabProtocolGil::callbackCMsg(const mit_msgs::MocapPosition::ConstPtr &msg, const int32_t &index)
{
  C_[index] = *msg;
}


void MatlabProtocolGil::callbackSMsg(const mit_msgs::MocapPosition::ConstPtr &msg, const int32_t &index)
{
  S_[index] = *msg;
}

void MatlabProtocolGil::callbackThetaMsg(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  T_ = *msg;
}
