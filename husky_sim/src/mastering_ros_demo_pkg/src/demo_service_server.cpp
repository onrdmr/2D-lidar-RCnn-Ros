/*
 * Copyright (C) 2015, Lentin Joseph and Qbotics Labs Inc.
 * Email id : qboticslabs@gmail.com
 *
 *
 * Copyright (C) 2020, Lentin Joseph and Qbotics Labs Inc. and Jonathan Cacace.
 * Email id : qboticslabs@gmail.com jonathan.cacace@gmail.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
 */

#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <std_srvs/Empty.h>
#include <iostream>
#include <sstream>
#include "rosgraph_msgs/Clock.h"  // ros::Time::now() used
#include <thread>

using namespace std;

class MessageHandler
{
public:
  static void thread_demo_service_server(int argc, char** argv, ros::MultiThreadedSpinner spinner)
  {
    ros::init(argc, argv, "demo_service_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("demo_service", demo_service_callback);
    spinner.spin();
  }

  static void thread_demo_clock_subscriber(int argc, char** argv, ros::MultiThreadedSpinner spinner)
  {
    ros::init(argc, argv, "clock_subscriber");

    ros::NodeHandle node_obj;
    std::cout << "deneme " << std::endl;
    ros::Subscriber number_subscriber = node_obj.subscribe("/clock", 1, number_callback);
    spinner.spin();
  }

private:
  static void number_callback(const rosgraph_msgs::Clock::ConstPtr& msg)
  {
    // ROS_INFO_STREAM("Received " << msg->clock << ros::Time::now());

    std::stringstream str;
    str << msg->clock;
    double simTime = std::stod(str.str());
    if (simTime > 10)
    {
      ros::NodeHandle n;
      ros::ServiceClient resetGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
      std_srvs::Empty srv;

      if (resetGazebo.call(srv))
      {
        ROS_INFO_STREAM("Received " << msg->clock << ros::Time::now());
        ROS_INFO("Resettig Gazebo World");
        ros::Duration(1).sleep();
        // assign new positon or new map if all position are loaded
        // print qui way of how much map is printed in memory
      }
    }
  }
  static bool demo_service_callback(mastering_ros_demo_pkg::demo_srv::Request& req,
                                    mastering_ros_demo_pkg::demo_srv::Response& res)
  {
    std::stringstream ss;
    ss << "walls1";
    res.out = ss.str();

    ROS_INFO("From Client [%s], Server says [%s]", req.in.c_str(), res.out.c_str());

    // double simTime = ros::Time::now().toSec();
    // if (simTime > 20)
    // {
    //   ros::NodeHandle n;
    //   ros::ServiceClient resetGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    //   std_srvs::Empty srv;

    //   if (resetGazebo.call(srv))
    //   {
    //     ROS_INFO("Resettig Gazebo World");
    //   }
    // }
    return true;
  }

private:
  int wallSequenceId;
  int robotPositionId;
  int bitmapId;
  std::string mapType;
};

int main(int argc, char** argv)
{
  // ros::Rate r(1);
  ros::MultiThreadedSpinner spinner(2);
  ROS_INFO("Ready to receive from client.");
  // thread_demo_clock_subscriber(argc, argv);
  std::thread update(MessageHandler::thread_demo_clock_subscriber, argc, argv, spinner);
  std::thread server(MessageHandler::thread_demo_service_server, argc, argv, spinner);

  update.join();
  server.join();

  return 0;
}
