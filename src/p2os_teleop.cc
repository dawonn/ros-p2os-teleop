/*
 * teleop_base
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * modifications to teleop_base to work with p2os
 * Copyright (C) 2010  David Feil-Seifer   [dfseifer@usc.edu] 
 *                     Edward T. Kaszubski [kaszubsk@usc.edu]
 * Copyright (C) 2012  Dereck Wonnacott    [dereck@gmail.com]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *       
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *       
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */


#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include "ros/ros.h"

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class TeleopBase 
{
 public:
  geometry_msgs::Twist cmd, passthrough_cmd;
  double req_vx, req_vy, req_vw;
  double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
  int    axis_vx, axis_vy, axis_vw;
  int    deadman_button, run_button;
  bool   deadman_no_publish_;
  bool   deadman_;
  bool   running_;

  ros::Time       last_recieved_joy_message_time_;
  ros::Duration   joy_msg_timeout_;

  ros::NodeHandle n_;
  ros::Publisher  vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber passthrough_sub_;

  TeleopBase(bool deadman_no_publish = false) : 
    max_vx(0.6), 
    max_vy(0.6), 
    max_vw(0.8), 
    max_vx_run(1.2), 
    max_vy_run(1.2), 
    max_vw_run(1.2), 
    deadman_no_publish_(deadman_no_publish), 
    running_(false)
  { }

	~TeleopBase() 
	{ }

  void init()
  {
    double joy_msg_timeout;
    
    ros::NodeHandle private_nh("~");
       
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    private_nh.param("max_vx", max_vx, max_vx);
    private_nh.param("max_vy", max_vy, max_vy);
    private_nh.param("max_vw", max_vw, max_vw);

    // Set max speed while running
    private_nh.param("max_vx_run", max_vx_run, max_vx_run);
    private_nh.param("max_vy_run", max_vy_run, max_vy_run);
    private_nh.param("max_vw_run", max_vw_run, max_vw_run);

    private_nh.param("axis_vx", axis_vx, 3);
    private_nh.param("axis_vw", axis_vw, 0);
    private_nh.param("axis_vy", axis_vy, 2);

    private_nh.param("deadman_button", deadman_button, 0);
    private_nh.param("run_button", run_button, 0);
    
    private_nh.param("joy_msg_timeout", joy_msg_timeout, -1.0); //default to no timeout
    if (joy_msg_timeout <= 0)
    {
      joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
      ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
    }
    else
    {
      joy_msg_timeout_.fromSec(joy_msg_timeout);
      ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
    }

    ROS_DEBUG("max_vx: %.3f m/s\n", max_vx);
    ROS_DEBUG("max_vy: %.3f m/s\n", max_vy);
    ROS_DEBUG("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);

    ROS_DEBUG("max_vx_run: %.3f m/s\n", max_vx_run);
    ROS_DEBUG("max_vy_run: %.3f m/s\n", max_vy_run);
    ROS_DEBUG("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);

    ROS_DEBUG("axis_vx: %d\n", axis_vx);
    ROS_DEBUG("axis_vy: %d\n", axis_vy);
    ROS_DEBUG("axis_vw: %d\n", axis_vw);

    ROS_INFO("deadman_button: %d", deadman_button);
    ROS_INFO("run_button: %d", run_button);
    ROS_DEBUG("joy_msg_timeout: %f\n", joy_msg_timeout);

    vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    passthrough_sub_ = n_.subscribe( "des_vel", 10, &TeleopBase::passthrough_cb, this );
    joy_sub_ = n_.subscribe("joy", 10, &TeleopBase::joy_cb, this);
  }

  void passthrough_cb( const geometry_msgs::TwistConstPtr& pass_msg )
  {
    ROS_DEBUG( "passthrough_cmd: [%f,%f]", passthrough_cmd.linear.x, passthrough_cmd.angular.z );
    passthrough_cmd = *pass_msg;
  }

  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {  
    // Deadman switch (Motion Enable)
    if(deadman_button >= 0 && (unsigned int)deadman_button > joy_msg->buttons.size())
    {
      ROS_ERROR_ONCE("Deadman button (%d) is out of range (%d).", deadman_button, (int)joy_msg->buttons.size());
      return;
    }	    
    deadman_ = joy_msg->buttons[deadman_button];
    if (!deadman_) return;

    // Running (high speed mode)
    if(run_button >= 0 && (unsigned int)run_button < joy_msg->buttons.size())
    {
      running_ = (joy_msg->buttons[run_button]);
    }
    else
    {
      ROS_ERROR_ONCE("Running button (%d) is out of range (%d).", deadman_button, (int)joy_msg->buttons.size());
      running_ = 0;
    }
    double vx = running_ ? max_vx_run : max_vx;
    double vy = running_ ? max_vy_run : max_vy;
    double vw = running_ ? max_vw_run : max_vw;

    // Joystick Axis mapping
    if((axis_vx >= 0) && ((unsigned int)axis_vx < joy_msg->axes.size()))
    {
       req_vx = joy_msg->axes[axis_vx] * vx;
    }
    else
    {
       ROS_ERROR_ONCE("axis_vx (%d) is out of range (%d).", axis_vx, (int)joy_msg->axes.size());
       req_vx = 0.0;
    }
       
    if((axis_vy >= 0) && ((unsigned int)axis_vy < joy_msg->axes.size()))
    {
       req_vy = joy_msg->axes[axis_vy] * vy;
    }
    else
    {
       ROS_ERROR_ONCE("axis_vy (%d) is out of range (%d).", axis_vy, (int)joy_msg->axes.size());
       req_vy = 0.0;
    }
       
    if((axis_vw >= 0) && ((unsigned int)axis_vw < joy_msg->axes.size()))
    {
       req_vw = joy_msg->axes[axis_vw] * vw;
    }
    else
    {
       ROS_ERROR_ONCE("axis_vw (%d) is out of range (%d).", axis_vw, (int)joy_msg->axes.size());
       req_vw = 0.0;
    }
    
    // Record this message reciept
    last_recieved_joy_message_time_ = ros::Time::now();
  }

  void send_cmd_vel()
  {
    if(deadman_ && (last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now()) )
    {
      cmd.linear.x  = req_vx;
      cmd.linear.y  = req_vy;
      cmd.angular.z = req_vw;
      vel_pub_.publish(cmd);

      fprintf(stdout, "teleop_base:: %f, %f, %f\n", cmd.linear.x, cmd.linear.y, cmd.angular.z);
    }
    else
    {
      // cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
      cmd = passthrough_cmd;
      if (!deadman_no_publish_) vel_pub_.publish(cmd);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_base");
  ros::NodeHandle nh;
  const char* opt_no_publish = "--deadman_no_publish";

  bool no_publish = false;
  for(int i=1;i<argc;i++)
  {
    if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
      no_publish = true;
  }

  ros::Rate pub_rate(20);

  TeleopBase teleop_base(no_publish);
  teleop_base.init();

  while (ros::ok())
  {
    ros::spinOnce(); 
    teleop_base.send_cmd_vel();
    pub_rate.sleep();
  }

  exit(0);
  return 0;
}


