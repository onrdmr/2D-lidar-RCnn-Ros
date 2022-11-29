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
#include <filesystem>
#include <map>

using namespace std;
namespace fs = std::filesystem;

class MessageHandler
{
public:
  MessageHandler(std::string buildingPath) : buildingEditorPath(buildingPath)
  {
    std::cout << "building editor path is loaded : " << buildingPath << std::endl;

    // std::map<long int, fs::path> dataset;
    for (const auto& entry : fs::directory_iterator(buildingPath))
    {
      // std::cout << entry << std::endl;
      std::stringstream s;
      s << entry;
      std::string str = s.str();
      std::string folder_name = "wall";
      std::size_t found = str.find(folder_name);

      if (found != string::npos)
      {
        int key = std::stoi(str.substr(found + folder_name.size()));
        // std::cout << key << std::endl;
        // std::cout << str.at(sequenceWall) << std::endl;
        dataset.insert({ key, entry });
      }
    }

    for (auto itr = dataset.begin(); itr != dataset.end(); ++itr)
    {
      std::cout << "loading dataset catalog to server." << std::endl;
      std::cout << itr->second << std::endl;
    }

    std::cout << "Total dataset count : " << dataset.size() << std::endl;

    this->totalWallSequence = dataset.size();
    this->wallSequenceId = 1;
    this->robotPositionId = 0;
    this->dbItr = dataset.begin();
    this->mapType = "single-room";
    this->bitmapId = -1;

    this->totalRobotPosition = 0;
    this->resetTime = 10;

    set_message();

    printMessageDebug();
  }

public:
  void demo_service_server(int argc, char** argv, ros::MultiThreadedSpinner spinner)
  {
    ros::init(argc, argv, "demo_service_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("demo_service", &MessageHandler::demo_service_callback, this);
    spinner.spin();
  }

  void demo_clock_subscriber(int argc, char** argv, ros::MultiThreadedSpinner spinner)
  {
    ros::init(argc, argv, "clock_subscriber");

    ros::NodeHandle node_obj;
    std::cout << "deneme " << std::endl;
    ros::Subscriber number_subscriber = node_obj.subscribe("/clock", 1, &MessageHandler::reset_sim_callback, this);
    spinner.spin();
  }

  std::thread demo_clock_subscriber_thread(int argc, char** argv, ros::MultiThreadedSpinner spinner)
  {
    return std::thread([=] { demo_clock_subscriber(argc, argv, spinner); });
  }

  std::thread demo_service_server_thread(int argc, char** argv, ros::MultiThreadedSpinner spinner)
  {
    return std::thread([=] { demo_service_server(argc, argv, spinner); });
  }

private:
  void set_message()
  {
    bool singleRoom = true;
    std::cout << "setting message " << this->dbItr->second << std::endl;
    this->totalRobotPosition = 0;
    for (const auto& entry : fs::directory_iterator(this->dbItr->second))
    {
      std::cout << entry << std::endl;
      std::stringstream s;
      s << entry;
      std::string str = s.str();

      if (str.find("mmap") != string::npos)
      {
        singleRoom = false;
        this->mapType = "multi-room";
        std::cout << "multi-room" << std::endl;
      }
      if (str.find("sub_models") != string::npos)
      {
        std::cout << "submodels" << std::endl;
        this->bitmapId = 0;

        this->totalSubModel = 0;
        std::filesystem::path p1{ str.substr(1, str.size() - 2) };
        for (auto& p : std::filesystem::directory_iterator(p1))
        {
          this->totalSubModel++;
        }
        this->totalSubModel = this->totalSubModel / 2;
      }
      std::string robotPositionStr = "robot_position_";
      int found = str.find(robotPositionStr);
      if (found != string::npos)
      {
        int positionIdx = std::stoi(str.substr(found + robotPositionStr.size()));
        if (positionIdx > this->totalRobotPosition)
        {
          this->totalRobotPosition = positionIdx;

          if (this->totalSubModel != 0)
          {
            this->bitmapId = this->robotPositionId;
          }
          else
          {
            this->bitmapId = -1;
          }
        }
      }
    }
    if (singleRoom == true)
    {
      std::cout << "single-room" << std::endl;
      this->mapType = "single-room";
    }
  }
  void reset_sim_callback(const rosgraph_msgs::Clock::ConstPtr& msg)
  {
    // ROS_INFO_STREAM("Received " << msg->clock << ros::Time::now());

    std::stringstream str;
    str << msg->clock;
    double simTime = std::stod(str.str());
    if (this->resetTime != 1 && simTime > this->resetTime)  // ilk koşul exploration scan den dur mesajı gelirse
    {
      ros::NodeHandle n;
      ros::ServiceClient resetGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
      std_srvs::Empty srv;

      // std::cout << mapType << " " << wallSequenceId << " " << robotPositionId << "" << bitmapId << " " << std::endl;
      if (resetGazebo.call(srv))
      {
        ROS_INFO_STREAM("Received " << msg->clock << ros::Time::now());
        ROS_INFO("Resetting Gazebo World");
        setNewExplorationStates();
        ros::Duration(1).sleep();

        // assign new positon or new map if all position are loaded
        // print qui way of how much map is printed in memory
      }
    }
  }
  bool demo_service_callback(mastering_ros_demo_pkg::demo_srv::Request& req,
                             mastering_ros_demo_pkg::demo_srv::Response& res)
  {
    std::stringstream ss;

    res.wallSequence = this->wallSequenceId;
    res.robotPositionId = this->robotPositionId;
    res.bitmapId = this->bitmapId;
    res.mapType = this->mapType;
    res.buildingEditorPath = this->buildingEditorPath;

    ROS_INFO("IN:SERVER || From Client [%s], Server says [%s]", req.in.c_str(), res.mapType.c_str());

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
  void printMessageDebug()
  {
    std::cout << mapType << " wallSequenceId :" << this->wallSequenceId << " robotPositionId :" << this->robotPositionId
              << "  totalRobotPosition :" << this->totalRobotPosition
              << " totalWallSequence :" << this->totalWallSequence << " totalSubModel :" << this->totalSubModel
              << " bitmapId" << this->bitmapId << std::endl;
  }
  void setNewExplorationStates()
  {
    if (this->wallSequenceId == this->totalWallSequence)
    {
      std::cout << "program sonlandı yeni veri girdi kuyruğu beklenmekte" << std::endl;
      std::cout << "girdi verilirse tetiklenip toplan duvar sequence bilgisi güncellenecek." << std::endl;
      return;
    }
    if (this->bitmapId == -1)
    {
      if (this->robotPositionId < this->totalRobotPosition)
      {
        this->robotPositionId++;
      }
      else
      {
        std::cout << "state exploration." << std::endl;
        this->wallSequenceId++;
        this->robotPositionId = 0;

        this->dbItr++;
        std::cout << "itr" << this->dbItr->second << " " << this->wallSequenceId << " " << this->robotPositionId
                  << std::endl;

        set_message();
      }
    }
    else
    {
      if (this->robotPositionId < this->totalRobotPosition)
      {
        this->robotPositionId++;
        this->bitmapId++;
      }
      else
      {
        std::cout << "state exploration." << std::endl;
        this->wallSequenceId++;
        this->robotPositionId = 0;

        this->dbItr++;
        std::cout << "itr" << this->dbItr->second << " " << this->wallSequenceId << " " << this->robotPositionId
                  << std::endl;

        set_message();
      }
    }
    printMessageDebug();
  }

private:
  int wallSequenceId;
  int totalWallSequence;
  int totalSubModel;
  int robotPositionId;
  int bitmapId;
  int totalRobotPosition;
  int resetTime;
  std::string mapType;
  std::string buildingEditorPath;
  std::map<long int, fs::path> dataset;
  std::map<long int, fs::path>::iterator dbItr;
};

int main(int argc, char** argv)
{
  // ros::Rate r(1);
  ros::MultiThreadedSpinner spinner(2);
  MessageHandler* handler = new MessageHandler("/home/onur/building_editor_models");

  ROS_INFO("Ready to receive from client.");
  // thread_demo_clock_subscriber(argc, argv);
  std::thread update = handler->demo_clock_subscriber_thread(argc, argv, spinner);
  std::thread server = handler->demo_service_server_thread(argc, argv, spinner);
  // handler->std::thread server(&handler->thread_demo_service_server, argc, argv, spinner);

  update.join();
  server.join();

  return 0;
}
