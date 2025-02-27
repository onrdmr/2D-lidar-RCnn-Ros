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
    ROS_INFO_STREAM("SERVICE : Building editor path is loaded : " << buildingPath);

    this->recordSubF = NULL;
    this->recordSubR = NULL;
    this->recordSubOdom = NULL;
    this->readyToExplore = true;

    // std::map<long int, fs::path> dataset;
    for (const auto& entry : fs::directory_iterator(buildingPath))
    {
      // std::cout << entry << std::endl;
      std::stringstream s;
      s << entry;
      std::string str = s.str();
      std::string folder_name = "wall";
      std::size_t found = str.find(folder_name);

      ROS_INFO_STREAM("SERVICE : " << found << ": " << str << " " << str.substr(found + folder_name.size()));

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
      ROS_INFO("SERVICE : loading dataset catalog to server.");
      ROS_INFO_STREAM("SERVICE : " << itr->second);
    }

    ROS_INFO_STREAM("SERVICE : Total dataset count : " << dataset.size());

    this->totalWallSequence = dataset.size();
    this->wallSequenceId = 1;
    this->robotPositionId = 0;
    this->dbItr = dataset.begin();
    this->mapType = "single-room";
    this->bitmapId = -1;

    this->totalRobotPosition = 0;
    this->resetTime = 15;

    set_message();
  }

public:
  void demo_service_server(int argc, char** argv)
  {
    ros::MultiThreadedSpinner spinner(2);
    ros::init(argc, argv, "demo_service_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("demo_service", &MessageHandler::demo_service_callback, this);
    spinner.spin();
  }

  void demo_clock_subscriber(int argc, char** argv)
  {
    ros::init(argc, argv, "clock_subscriber");
    ros::MultiThreadedSpinner spinner(1);
    ros::NodeHandle node_obj;
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

  std::thread demo_clock_subscriber_thread(int argc, char** argv)
  {
    return std::thread([=] { demo_clock_subscriber(argc, argv); });
  }

  std::thread demo_service_server_thread(int argc, char** argv)
  {
    return std::thread([=] { demo_service_server(argc, argv); });
  }

private:
  void set_message()
  {
    bool singleRoom = true;
    ROS_INFO_STREAM("SERVICE : Setting message " << this->dbItr->second);
    this->totalRobotPosition = 0;
    this->totalSubModel = 0;
    for (const auto& entry : fs::directory_iterator(this->dbItr->second))
    {
      ROS_INFO_STREAM(entry);
      std::stringstream s;
      s << entry;
      std::string str = s.str();

      if (str.find("mmap") != string::npos)
      {
        singleRoom = false;
        this->mapType = "multi-room";
        ROS_INFO("SERVICE : multi-room");
      }
      if (str.find("sub_models") != string::npos)
      {
        ROS_INFO("SERVICE : submodels");
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
      ROS_INFO("single-room");
      this->mapType = "single-room";
    }
  }
  void reset_sim_callback(const rosgraph_msgs::Clock::ConstPtr& msg)
  {
    // const std::lock_guard<std::mutex> lock(mutex);
    std::stringstream str;
    str << msg->clock;
    double simTime = std::stod(str.str());
    // ROS_INFO_STREAM("resetTime :" << this->resetTime << "sim time:" << simTime);
    if (this->resetTime != -1 && simTime > this->resetTime)  // ilk koşul exploration scan den dur mesajı gelirse
    {
      const std::lock_guard<std::mutex> lock(mutex);

      if (this->recordSubF != NULL)
      {
        ROS_INFO("thread sonlandır.");
        this->recordSubF->unsubscribe();
        this->recordSubOdom->unsubscribe();
        // this->recordSubR->unsubscribe();

        this->recordSubF = NULL;
        // this->recordSubR = NULL;
        this->recordSubOdom = NULL;

        ROS_INFO("SERVICE : spin kırıldı. Record yazma sonlandı.");
      }
      ros::NodeHandle n;
      ros::ServiceClient resetGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
      std_srvs::Empty srv;
      // std::cout << mapType << " " << wallSequenceId << " " << robotPositionId << "" << bitmapId << " " << std::endl;
      ROS_INFO_STREAM("SERVICE : Received " << msg->clock << " " << ros::Time::now());
      ROS_INFO("SERVICE : Resetting Gazebo World");
      if (resetGazebo.call(srv))
      {
        ROS_INFO("SERVICE : Waiting for duration.");
        ros::Duration(1).sleep();

        // ros::getGlobalCallbackQueue()->clear();

        // assign new positon or new map if all position are loaded
        // print qui way of how much map is printed in memory
      }
      // setNewExplorationStates();
    }
  }

  void writeBagFilesToWall()
  {
    std::string sensorDataPath = this->buildingEditorPath + "/wall" + std::to_string(this->wallSequenceId) + "/sensors";

    ROS_INFO_STREAM(sensorDataPath << " : " << this->defaultBagPath);
    if (!boost::filesystem::exists(sensorDataPath))
    {
      boost::filesystem::create_directory(sensorDataPath);
    }

    if (!boost::filesystem::exists(sensorDataPath + "/" + std::to_string(this->robotPositionId)))
    {
      ROS_INFO_STREAM("SERVICE : yazılacak bag path: " << this->defaultBagPath);
      boost::filesystem::create_directory(sensorDataPath + "/" + std::to_string(this->robotPositionId));

      boost::filesystem::rename(this->defaultBagPath + this->filenameFrontLaserBag,
                                sensorDataPath + "/" + std::to_string(this->robotPositionId) + "/" +
                                    this->filenameFrontLaserBag);
      // boost::filesystem::rename(this->defaultBagPath + this->filenameRearLaserBag,
      //                           sensorDataPath + "/" + std::to_string(this->robotPositionId) + "/" +
      //                               this->filenameRearLaserBag);
      boost::filesystem::rename(this->defaultBagPath + this->filenameOdomBag,
                                sensorDataPath + "/" + std::to_string(this->robotPositionId) + "/" +
                                    this->filenameOdomBag);
    }
    else
    {
      ROS_INFO("SERVICE : dosya zaten var");
    }
  }

  void sync_callback(  // const sensor_msgs::LaserScan::ConstPtr& laserScanR,
      const sensor_msgs::LaserScan::ConstPtr& laserScanF, const nav_msgs::Odometry::ConstPtr& odom)
  {
    ROS_INFO("SERVICE : veriler bag olarak kaydediliyor. Append");

    rosbag::Bag bagF(this->filenameFrontLaserBag, rosbag::bagmode::BagMode::Append);
    // rosbag::Bag bagR(this->filenameRearLaserBag, rosbag::bagmode::BagMode::Append);
    rosbag::Bag bagO(this->filenameOdomBag, rosbag::bagmode::BagMode::Append);
    // std::cout << "burada kaydetme olacak" << std::endl;
    try
    {
      bagF.write("/front/scan", ros::Time::now(), laserScanF);

      // bagR.write("/rear/scan", ros::Time::now(), laserScanR);

      bagO.write("/ground_truth/state", ros::Time::now(), odom);
    }
    catch (rosbag::BagIOException&)
    {
      ROS_INFO("Exception occured this sequence.");
    }
    catch (rosbag::BagUnindexedException&)
    {
      ROS_INFO("Bag unindexed exception.");
    }
    // bagF.close();
    // bagO.close();
    // bagR.close();
  }

  void recordData()
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "record_node");
    // this->recordDataAvailable = new Available();

    this->recordSpinner = new ros::AsyncSpinner(1);
    ros::NodeHandle n;
    ROS_INFO("SERVICE : record data sleep 1 duration");
    while (!ros::Duration(1).sleep())
    {
    }

    ROS_INFO("SERVICE : Bag dosyası oluşturuluyor.");
    // ön lazer , alt lazer
    rosbag::Bag bag(this->filenameFrontLaserBag, rosbag::bagmode::BagMode::Write);
    // rosbag::Bag bag2(this->filenameRearLaserBag, rosbag::bagmode::BagMode::Write);
    rosbag::Bag bag3(this->filenameOdomBag, rosbag::bagmode::BagMode::Write);

    bag.close();
    // bag2.close();
    bag3.close();

    this->recordSubF = new message_filters::Subscriber<sensor_msgs::LaserScan>(n, "/front/scan", 1000);
    // this->recordSubR = new message_filters::Subscriber<sensor_msgs::LaserScan>(n, "/rear/scan", 1000);
    this->recordSubOdom = new message_filters::Subscriber<nav_msgs::Odometry>(n, "/ground_truth/state", 1000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(1000), *this->recordSubF  //, *this->recordSubR
                                                   ,
                                                   *this->recordSubOdom);
    sync.registerCallback(boost::bind(&MessageHandler::sync_callback, this, _1, _2));
    // this->recordSubR = n.subscribe("/clock", 100, &MessageHandler::clockFoo, this);
    // this->recordSubF = n.subscribe("/front/scan", 100, &MessageHandler::laserScanCallbackF, this);
    // this->recordSubF = n.subscribe("/odometry/filtered", 100, &MessageHandler::laserScanCallbackOdom, this);
    sync.init();

    // this->recordSpinner->start();
    // ros::waitForShutdown();
    recordSpin(0.01);

    ROS_INFO("SERVICE : çağrı thread ros spin kırıldı sonlandırır");
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
    ROS_INFO("SERVICE : spinning while recording");
  }

  bool demo_service_callback(mastering_ros_demo_pkg::demo_srv::Request& req,
                             mastering_ros_demo_pkg::demo_srv::Response& res)
  {
    if (req.in == "SET_MAP")
    {
      ROS_INFO("SERVICE : send req içindeyim ");
      std::stringstream ss;

      res.wallSequence = this->wallSequenceId;
      res.robotPositionId = this->robotPositionId;

      res.bitmapId = findBitmapId();  // sub_models yoksa -1 olmalı

      res.mapType = this->mapType;
      res.buildingEditorPath = this->buildingEditorPath;
      res.out = "SET_MAP";
      this->readyToExplore = false;
      ROS_INFO("SERVICE : IN:SERVER || From Client [%s], Server says [%d] [%d] [%d] [%s] [%s]", req.in.c_str(),
               res.wallSequence, res.robotPositionId, res.bitmapId, res.mapType.c_str(),
               res.buildingEditorPath.c_str());
    }
    else if (req.in == "READY_TO_EXPLORE")
    {
      // this->readyToExplore = true;
      ROS_INFO("SERVICE : Record thread created");
      this->recordThread = std::thread([=] { this->recordData(); });
      this->recordThread.detach();
    }
    else if (req.in == "SEND_REQ")
    {
      res.wallSequence = this->wallSequenceId;
      res.robotPositionId = this->robotPositionId;
      res.bitmapId = this->bitmapId;
      res.mapType = this->mapType;
      res.buildingEditorPath = this->buildingEditorPath;
      res.out = (this->readyToExplore) ? "true" : "false";

      ROS_INFO("SERVICE : IN:SERVER || From Client [%s], Server says [%d] [%d] [%d] [%s] [%s]", req.in.c_str(),
               res.wallSequence, res.robotPositionId, res.bitmapId, res.mapType.c_str(),
               res.buildingEditorPath.c_str());
    }

    else if (req.in == "REMOVE_RESET")
    {
      auto thread = std::thread([=] {
        ros::NodeHandle n;
        ros::Duration(0.5).sleep();

        ros::ServiceClient resetGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
        std_srvs::Empty srv;
        // res.out = "SET_MAP";
        setNewExplorationStates();
        this->readyToExplore = true;
        bool call = false;
        while (!call)
        {
          if (call = resetGazebo.call(srv))
          {
            ROS_INFO("SERVICE : Resetting Gazebo World with REMOVED MODELS");
          }
          else
          {
            resetGazebo.waitForExistence();
          }
        }
      });
      thread.detach();
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
    ROS_INFO_STREAM("SERVICE : " << mapType << " wallSequenceId :" << this->wallSequenceId << " robotPositionId :"
                                 << this->robotPositionId << "  totalRobotPosition :" << this->totalRobotPosition
                                 << " totalWallSequence :" << this->totalWallSequence
                                 << " totalSubModel :" << this->totalSubModel << " bitmapId" << this->bitmapId);
  }

  int findBitmapId()
  {
    std::string bitmapExists =
        this->buildingEditorPath + "/wall" + std::to_string(this->wallSequenceId) + "/sub_models";
    return (boost::filesystem::exists(bitmapExists)) ? this->bitmapId : -1;
  }

  void setNewExplorationStates()
  {
    this->bitmapId = findBitmapId();
    ROS_INFO("SERVICE : setNewExploration State this->bitmapId:[%d] this->totalRobotPosition:[%d]", this->bitmapId,
             this->totalRobotPosition);

    if (this->bitmapId == -1)
    {
      if (this->robotPositionId < this->totalRobotPosition)
      {
        ROS_INFO("robot Position id arttı setnewexpl");
        this->robotPositionId++;
      }
      else
      {
        ROS_INFO("state exploration.");
        this->wallSequenceId++;
        if (this->wallSequenceId == this->totalWallSequence + 1)
        {
          ROS_INFO("program sonlandı yeni veri girdi kuyruğu beklenmekte");
          ROS_INFO("girdi verilirse tetiklenip toplan duvar sequence bilgisi güncellenecek.");
          return;
        }
        this->robotPositionId = 0;

        this->dbItr++;
        ROS_INFO_STREAM("SERVICE : itr" << this->dbItr->second << " " << this->wallSequenceId << " "
                                        << this->robotPositionId);

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
        ROS_INFO("SERVICE : state exploration.");
        this->wallSequenceId++;

        if (this->wallSequenceId == this->totalWallSequence + 1)
        {
          ROS_INFO("SERVICE : program sonlandı yeni veri girdi kuyruğu beklenmekte");
          ROS_INFO("SERVICE : girdi verilirse tetiklenip toplan duvar sequence bilgisi güncellenecek.");
          return;
        }

        this->robotPositionId = 0;

        this->dbItr++;
        ROS_INFO_STREAM("SERVIC : itr" << this->dbItr->second << " " << this->wallSequenceId << " "
                                       << this->robotPositionId);

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
  // const std::string filenameRearLaserBag = "rear_scan.bag";
  const std::string filenameOdomBag = "odometry.bag";
  const std::string defaultBagPath = getHomeDirectory() + "/.ros/";

  // Available* recordDataAvailable;
};

int main(int argc, char** argv)
{
  // ros::Rate r(1);

  MessageHandler* handler = new MessageHandler("/home/onur/building_editor_models");

  ROS_INFO("SERVICE : Ready to receive from client.");
  // thread_demo_clock_subscriber(argc, argv);
  std::thread update = handler->demo_clock_subscriber_thread(argc, argv);
  std::thread server = handler->demo_service_server_thread(argc, argv);
  // handler->std::thread server(&handler->thread_demo_service_server, argc, argv, spinner);

  update.join();
  server.join();

  return 0;
}
