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
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <rosbag/bag.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// senkron subscriber ile yazılma yapılacak
// question 47406

#include <ros/callback_queue.h>
#include <sys/types.h>
#include <pwd.h>
#include <unistd.h>
using namespace std;
namespace fs = std::filesystem;

class MessageHandler
{
public:
  MessageHandler(std::string buildingPath) : buildingEditorPath(buildingPath)
  {
    std::cout << "building editor path is loaded : " << buildingPath << std::endl;

    this->recordSubF = NULL;
    this->recordSubR = NULL;
    this->recordSubOdom = NULL;
    // std::map<long int, fs::path> dataset;
    for (const auto& entry : fs::directory_iterator(buildingPath))
    {
      // std::cout << entry << std::endl;
      std::stringstream s;
      s << entry;
      std::string str = s.str();
      std::string folder_name = "wall";
      std::size_t found = str.find(folder_name);
      std::fstream a("/home/onur/2D-lidar-RCnn-Ros/husky_sim/log/log.log", std::ios::out | std::ios::in);
      a << found << ": " << str << " " << str.substr(found + folder_name.size()) << std::endl;
      a.close();
      std::cout << found << ": " << str << " " << str.substr(found + folder_name.size()) << std::endl;

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
    ros::SubscribeOptions ops;
    const boost::function<void(const rosgraph_msgs::Clock::ConstPtr&)> reset_sim_callback(
        boost::bind(&MessageHandler::reset_sim_callback, this, _1));
    ops.template init("/clock", 1, reset_sim_callback);
    ops.queue_size = 1;
    ops.callback_queue = NULL;

    // ops.allow_concurrent_callbacks = true;
    ros::Subscriber number_subscriber = node_obj.subscribe(ops);
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
        this->totalSubModel = 0;
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

          if (this->totalSubModel > 0)
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
    // const std::lock_guard<std::mutex> lock(mutex);
    std::stringstream str;
    str << msg->clock;
    double simTime = std::stod(str.str());
    if (this->resetTime != -1 && simTime > this->resetTime)  // ilk koşul exploration scan den dur mesajı gelirse
    {
      const std::lock_guard<std::mutex> lock(mutex);

      if (this->recordSubF != NULL)
      {
        std::cout << "thread sonlandır." << std::endl;
        this->recordSubF->unsubscribe();
        this->recordSubOdom->unsubscribe();
        this->recordSubR->unsubscribe();

        this->recordSubF = NULL;
        this->recordSubR = NULL;
        this->recordSubOdom = NULL;

        std::cout << "spin kırılıp yeni record lar yazılacak" << std::endl;
      }
      ros::NodeHandle n;
      ros::ServiceClient resetGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
      std_srvs::Empty srv;
      // std::cout << mapType << " " << wallSequenceId << " " << robotPositionId << "" << bitmapId << " " << std::endl;
      ROS_INFO_STREAM("Received " << msg->clock << " " << ros::Time::now());
      ROS_INFO("Resetting Gazebo World");
      if (resetGazebo.call(srv))
      {
        ROS_INFO("Waiting for duration.");
        ros::Duration(1).sleep();

        // ros::getGlobalCallbackQueue()->clear();

        // assign new positon or new map if all position are loaded
        // print qui way of how much map is printed in memory
      }
      setNewExplorationStates();
    }
  }

  void writeBagFilesToWall()
  {
    std::string sensorDataPath = this->buildingEditorPath + "/wall" + std::to_string(this->wallSequenceId) + "/sensors";
    std::fstream a("/home/onur/2D-lidar-RCnn-Ros/husky_sim/log/log.log", ios::out | ios::in);
    a << sensorDataPath << " : " << this->defaultBagPath << std::endl;
    a.close();
    std::cout << sensorDataPath << " : " << this->defaultBagPath << std::endl;
    if (!boost::filesystem::exists(sensorDataPath))
    {
      boost::filesystem::create_directory(sensorDataPath);
    }

    if (!boost::filesystem::exists(sensorDataPath + "/" + std::to_string(this->robotPositionId)))
    {
      std::cout << "yazılacak path: " << this->defaultBagPath << std::endl;
      a << this->defaultBagPath << std::endl;
      boost::filesystem::create_directory(sensorDataPath + "/" + std::to_string(this->robotPositionId));

      boost::filesystem::rename(this->defaultBagPath + this->filenameFrontLaserBag,
                                sensorDataPath + "/" + std::to_string(this->robotPositionId) + "/" +
                                    this->filenameFrontLaserBag);
      boost::filesystem::rename(this->defaultBagPath + this->filenameRearLaserBag,
                                sensorDataPath + "/" + std::to_string(this->robotPositionId) + "/" +
                                    this->filenameRearLaserBag);
      boost::filesystem::rename(this->defaultBagPath + this->filenameOdomBag,
                                sensorDataPath + "/" + std::to_string(this->robotPositionId) + "/" +
                                    this->filenameOdomBag);
    }
    else
    {
      ROS_INFO("dosya zaten var");
    }
    a.close();
  }

  void sync_callback(const sensor_msgs::LaserScan::ConstPtr& laserScanR,
                     const sensor_msgs::LaserScan::ConstPtr& laserScanF, const nav_msgs::Odometry::ConstPtr& odom)
  {
    // ROS_INFO("veriler bag olarak kaydediliyor.");

    rosbag::Bag bagF(this->filenameFrontLaserBag, rosbag::bagmode::BagMode::Append);
    rosbag::Bag bagR(this->filenameRearLaserBag, rosbag::bagmode::BagMode::Append);
    rosbag::Bag bagO(this->filenameOdomBag, rosbag::bagmode::BagMode::Append);
    // std::cout << "burada kaydetme olacak" << std::endl;
    try
    {
      bagF.write("/front/scan", ros::Time::now(), laserScanF);

      bagR.write("/rear/scan", ros::Time::now(), laserScanR);

      bagO.write("/odometry/filtered", ros::Time::now(), odom);
    }
    catch (rosbag::BagIOException&)
    {
      std::cout << "exception occured this sequence" << std::endl;
    }

    bagF.close();
    bagO.close();
    bagR.close();
  }

  void recordData()
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "record_node");
    // this->recordDataAvailable = new Available();

    this->recordSpinner = new ros::AsyncSpinner(1);
    ros::NodeHandle n;
    ros::Duration(1).sleep();

    // ön lazer , alt lazer
    rosbag::Bag bag(this->filenameFrontLaserBag, rosbag::bagmode::BagMode::Write);
    rosbag::Bag bag2(this->filenameRearLaserBag, rosbag::bagmode::BagMode::Write);
    rosbag::Bag bag3(this->filenameOdomBag, rosbag::bagmode::BagMode::Write);

    bag.close();
    bag2.close();
    bag3.close();

    this->recordSubF = new message_filters::Subscriber<sensor_msgs::LaserScan>(n, "/front/scan", 1000);
    this->recordSubR = new message_filters::Subscriber<sensor_msgs::LaserScan>(n, "/rear/scan", 1000);
    this->recordSubOdom = new message_filters::Subscriber<nav_msgs::Odometry>(n, "/odometry/filtered", 1000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan,
                                                            nav_msgs::Odometry>
        SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), *this->recordSubF, *this->recordSubR,
                                                   *this->recordSubOdom);
    sync.registerCallback(boost::bind(&MessageHandler::sync_callback, this, _1, _2, _3));
    // this->recordSubR = n.subscribe("/clock", 100, &MessageHandler::clockFoo, this);
    // this->recordSubF = n.subscribe("/front/scan", 100, &MessageHandler::laserScanCallbackF, this);
    // this->recordSubF = n.subscribe("/odometry/filtered", 100, &MessageHandler::laserScanCallbackOdom, this);
    sync.init();

    // this->recordSpinner->start();
    // ros::waitForShutdown();
    recordSpin(0.01);

    std::cout << "çağrı thread ros spin kırıldı sonlandırır" << std::endl;
  }

  void recordSpin(const float& duration)
  {
    while (ros::ok())
    {
      if (this->recordSubOdom != NULL)
      {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(duration));
      }
      else
      {
        writeBagFilesToWall();
        break;
      }
    }
  }

  void clockFoo(const rosgraph_msgs::Clock::ConstPtr& foo)
  {
    ROS_INFO("spinning while recording");
  }

  // void laserScanCallbackR(const sensor_msgs::LaserScan::ConstPtr& laserScanR)
  // {
  //   std::cout << "burada kaydetme olacak" << std::endl;
  //   rosbag::Bag bag("rear_scan.bag", rosbag::bagmode::BagMode::Append);
  //   bag.write("/rear/scan", ros::Time::now(), laserScanR);
  //   bag.close();
  // }
  // void laserScanCallbackF(const sensor_msgs::LaserScan::ConstPtr& laserScanF)
  // {
  //   std::cout << "burada kaydetme olacak" << std::endl;
  //   rosbag::Bag bag("front_scan.bag", rosbag::bagmode::BagMode::Append);
  //   bag.write("/rear/scan", ros::Time::now(), laserScanF);
  //   bag.close();
  // }
  // void laserScanCallbackOdom(const nav_msgs::Odometry::ConstPtr& odom)
  // {
  //   std::cout << "burada kaydetme olacak" << std::endl;
  //   rosbag::Bag bag("odometry.bag", rosbag::bagmode::BagMode::Append);
  //   bag.write("/odometry/filtered", ros::Time::now(), odom);
  //   bag.close();
  // }
  bool demo_service_callback(mastering_ros_demo_pkg::demo_srv::Request& req,
                             mastering_ros_demo_pkg::demo_srv::Response& res)
  {
    if (req.in == "SEND_REQ")
    {
      std::cout << "send req içindeyim " << std::endl;
      std::stringstream ss;

      res.wallSequence = this->wallSequenceId;
      res.robotPositionId = this->robotPositionId;
      res.bitmapId = this->bitmapId;
      res.mapType = this->mapType;
      res.buildingEditorPath = this->buildingEditorPath;
      res.out = "NOT_REMOVED";
      this->readyToExplore = false;
      ROS_INFO("IN:SERVER || From Client [%s], Server says [%d] [%d] [%d] [%s] [%s]", req.in.c_str(), res.wallSequence,
               res.robotPositionId, res.bitmapId, res.mapType.c_str(), res.buildingEditorPath.c_str());
    }
    else if (req.in == "READY_TO_EXPLORE")
    {
      this->readyToExplore = true;
      // std::cout << "Not Implemented yet" << std::endl;
      this->recordThread = std::thread([=] { this->recordData(); });
      this->recordThread.detach();
    }
    else if (req.in == "EXPLORE")
    {
    }

    else if (req.in == "REMOVE_MODELS")
    {
      ros::NodeHandle n;
      ros::ServiceClient resetGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
      std_srvs::Empty srv;
      res.out = "REMOVED";
      if (resetGazebo.call(srv))
      {
        ROS_INFO("Resettig Gazebo World with REMOVED MODELS");
      }
    }
    return true;
  }

private:
  const std::string getHomeDirectory()
  {
    const char* homedir;
    if ((homedir = getenv("HOME")) == NULL)
    {
      homedir = getpwuid(getuid())->pw_dir;
    }
    return std::string(homedir);
  }

  void printMessageDebug()
  {
    std::cout << mapType << " wallSequenceId :" << this->wallSequenceId << " robotPositionId :" << this->robotPositionId
              << "  totalRobotPosition :" << this->totalRobotPosition
              << " totalWallSequence :" << this->totalWallSequence << " totalSubModel :" << this->totalSubModel
              << " bitmapId" << this->bitmapId << std::endl;
  }
  void setNewExplorationStates()
  {
    std::cout << "setNewExploration State" << std::endl;
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
        std::cout << "robot Position id arttı setnewexpl" << std::endl;
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
  bool readyToExplore;  // std::mutex explore;
  std::mutex mutex;
  std::string mapType;
  std::string buildingEditorPath;
  std::map<long int, fs::path> dataset;
  std::map<long int, fs::path>::iterator dbItr;
  message_filters::Subscriber<sensor_msgs::LaserScan>* recordSubR;
  message_filters::Subscriber<sensor_msgs::LaserScan>* recordSubF;
  message_filters::Subscriber<nav_msgs::Odometry>* recordSubOdom;
  ros::AsyncSpinner* recordSpinner;
  std::thread recordThread;
  const std::string filenameFrontLaserBag = "front_scan.bag";
  const std::string filenameRearLaserBag = "rear_scan.bag";
  const std::string filenameOdomBag = "odometry.bag";
  const std::string defaultBagPath = getHomeDirectory() + "/.ros/";

  // Available* recordDataAvailable;
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
