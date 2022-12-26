#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include <filesystem>
#include <map>
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

#include <unistd.h>

// using namespace std;
namespace fs = std::filesystem;

namespace gazebo
{
class Factory : public WorldPlugin
{
public:
  void Init()
  {
    // in the first reset subscribe to service
  }

  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    this->buildingEditorPath = "/home/onur/building_editor_models";

    ROS_INFO_STREAM("Factory World Plugin Loaded : World " << _parent->Name());
    this->worldPtr = _parent;
    this->sdf = _sdf;
    this->request = "SET_MAP";
    // Option 3: Insert model from file via message passing.
    // insertModelViaMP(_parent);

    // worldPtr->StartTime();
  }

  void RemoveModels()
  {
    physics::Model_V models = worldPtr->Models();  // modelptr vector

    bool pauseState = worldPtr->IsPaused();
    worldPtr->SetPaused(true);

    for (physics::ModelPtr model : models)
    {
      std::string modelName = model->GetName();
      if (modelName != "ground_plane" && modelName != "husky")
      {
        worldPtr->RemoveModel(model);
      }
    }
    models.clear();

    worldPtr->SetPaused(pauseState);
  }

  void insertWall()
  {
    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(worldPtr->Name());

    // Create a publisher on the ~/factory topic
    transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

    // Create the message
    msgs::Factory msg;

    // Model file to load
    const std::string str = "model://wall" + std::to_string(modelWallSequence);

    // int removedWallSequence = modelWallSequence - 1;
    // const std::string removeModel = "walls" + std::to_string(removedWallSequence);
    // std::cout << "removedWallSequence: " << removeModel << std::endl;

    // worldPtr->RemoveModel(removeModel);
    // std::cout << str << " loading." << std::endl;

    msg.set_sdf_filename(str);

    // Pose to initialize the model to
    // msgs::Set(msg.mutable_pose(),
    //           ignition::math::Pose3d(ignition::math::Vector3d(0, 0, 0), ignition::math::Quaterniond(0, 0, 0)));

    // Send the message
    factoryPub->Publish(msg);
    ROS_INFO_STREAM("FACTORY : Model with modelWallSequence " << modelWallSequence << " is loaded ");
  }

  void insertWallWithRemoval()
  {
    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(worldPtr->Name());

    // Create a publisher on the ~/factory topic
    transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

    // Create the message
    msgs::Factory msg;

    // Model file to load
    const std::string str = "model://wall" + std::to_string(modelWallSequence);

    if (worldPtr->ModelByName("walls" + std::to_string(modelWallSequence)) == NULL)
    {
      int removedWallSequence = modelWallSequence - 1;
      const std::string removeModel = "walls" + std::to_string(removedWallSequence);

      // std::cout << "removedWallSequence: " << removeModel << std::endl;
      if (worldPtr->ModelByName(removeModel) != NULL)
      {
        physics::ModelPtr modelPtr = worldPtr->ModelByName(removeModel);
        modelPtr->SetName("removed");
        modelPtr->SetSelected(true);
        worldPtr->RemoveModel(modelPtr);
      }

      // physics::ModelPtr wallModel = worldPtr->ModelByName(str);
      // wallModel->SetName("removed");
      // wallModel->Fini();
      ROS_INFO_STREAM(str << " loading.");

      msg.set_sdf_filename(str);

      // Pose to initialize the model to
      msgs::Set(msg.mutable_pose(),
                ignition::math::Pose3d(ignition::math::Vector3d(0, 0, 0), ignition::math::Quaterniond(0, 0, 0)));

      // Send the message
      factoryPub->Publish(msg);
      ROS_INFO_STREAM("FACTORY : Model with modelWallSequence " << modelWallSequence << " is loaded ");
    }
  }

  void Reset()
  {
    ROS_INFO("FACTORY : World is resetting ");
    worldPtr->ResetTime();
    worldPtr->SetPaused(true);

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mastering_ros_demo_pkg::demo_srv>("demo_service");
    mastering_ros_demo_pkg::demo_srv srv;
    std::stringstream ss;

    ss.clear();
    ss << this->request;
    srv.request.in = ss.str();
    ROS_INFO_STREAM("Factory - sending request to server " << ss.str());
    bool call = true;
    ROS_INFO_STREAM("FACTORY : client states " << client.isPersistent() << " " << client.exists());

    while (call = client.call(srv))
    {
      ROS_INFO("FACTORY : Server response got");
      ROS_INFO("FACTORY : IN:CLIENT || From Client [%s], Server says [%d] [%d] [%d] [%s] [%s][%s]",
               srv.request.in.c_str(), srv.response.wallSequence, srv.response.robotPositionId, srv.response.bitmapId,
               srv.response.mapType.c_str(), srv.response.buildingEditorPath.c_str(), srv.response.out.c_str());
      if (call)
      {
        break;
      }
      else
      {
        client.waitForExistence();
      }
    }

    if (this->request == "REMOVE_RESET")
    {
      ROS_INFO("FACTORY : REMOVE is processing...");
      RemoveModels();
      this->request = "SET_MAP";

      // std::string removeReqReset = "REMOVE_RESET";
      // srv.request.in = removeReqReset;
      // if (client.call(srv))
      // {
      //   std::cout << "Factory - sending request to server " << removeReqReset << std::endl;
      // }
    }
    else if (this->request == "SET_MAP")
    {
      ROS_INFO("FACTORY : Setting map is processing...");

      this->modelWallSequence = srv.response.wallSequence;
      this->buildingEditorPath = srv.response.buildingEditorPath;
      if (srv.response.bitmapId != -1)
      {
        std::ifstream stream(this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/bitmap_" +
                             std::to_string(srv.response.bitmapId));
        std::stringstream sstream;
        sstream << stream.rdbuf();
        std::string bitmap = sstream.str();
        ROS_INFO_STREAM("FACTORY : bitmap " << bitmap);
        std::vector<std::string> token;
        boost::split(token, bitmap, boost::is_any_of(" "));

        std::string subModelPath =
            this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/sub_models/";

        ROS_INFO_STREAM("FACTORY : wall0 exception" << subModelPath);

        std::map<std::string, fs::path> modelSet;
        for (const auto& entry : fs::directory_iterator(subModelPath))
        {
          // std::cout << entry << std::endl;
          std::stringstream s;
          s << entry;
          std::string key = s.str();
          key.erase(key.begin());
          key.erase(key.end() - 1);
          // std::cout << key << std::endl;
          if (boost::algorithm::ends_with(key, ".sdf"))
          {
            std::vector<std::string> result;
            boost::split(result, key, boost::is_any_of("/"));
            std::string modelName = result[result.size() - 1].substr(0, result[result.size() - 1].size() - 4);

            // std::cout << " modelName : " << modelName << " key: " << key << std::endl;
            // std::cout << str.at(sequenceWall) << std::endl;
            modelSet.insert({ modelName, key });
          }
        }

        insertWall();

        int i = 0;
        for (auto itr = modelSet.begin(); itr != modelSet.end() && i < token.size(); ++itr, ++i)
        {
          ROS_INFO_STREAM("FACTORY : bitmap token :" << token[i] << " " << srv.response.bitmapId);
          if (std::stoi(token[i]) == true)
          {
            ROS_INFO_STREAM("FACTORY : written " << itr->second);
            sdf::SDF model;

            std::ifstream ifSdfStream(itr->second, std::ifstream::in);
            std::stringstream sdfStream;
            std::string sdf;
            sdfStream << ifSdfStream.rdbuf();
            sdf = sdfStream.str();

            model.SetFromString(sdf);
            worldPtr->InsertModelSDF(model);
            ifSdfStream.close();
          }
          else
          {
            continue;
          }
        }
      }
      else
      {
        insertWall();
      }

      srv.request.in = "READY_TO_EXPLORE";

      while (call = client.call(srv))
      {
        if (call)
        {
          worldPtr->SetPaused(false);
          break;
        }
        else
        {
          client.waitForExistence();
        }
      }
      // if (client.call(srv))
      // {
      //   worldPtr->SetPaused(false);
      // }

      this->request = "REMOVE_RESET";
    }
    else
    {
      ROS_INFO("FACTORY : Illegal response token first logic here for debug purpose");

      // if (client.call(srv))
      // {
      /*
        int32 wallSequence
        int32 robotPositionId
        int32 bitmapId
        string mapType
        string buildingEditorPath
      */

      // exploration bir şey isteyebilir. onun için 1 tane daha yer olacak server için.
      ROS_INFO("FACTORY : IN:CLIENT || From Client [%s], Server says [%d] [%d] [%d] [%s] [%s]", srv.request.in.c_str(),
               srv.response.wallSequence, srv.response.robotPositionId, srv.response.bitmapId,
               srv.response.mapType.c_str(), srv.response.buildingEditorPath.c_str());

      this->modelWallSequence = srv.response.wallSequence;
      this->buildingEditorPath = srv.response.buildingEditorPath;
      if (srv.response.bitmapId != -1)
      {
        // ROS_INFO("Factory says I will produce ")
        std::ifstream stream(this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/bitmap_" +
                             std::to_string(srv.response.bitmapId));
        std::stringstream sstream;
        sstream << stream.rdbuf();
        std::string bitmap = sstream.str();
        ROS_INFO_STREAM("FACTORY : Bitmap " << bitmap);
        std::vector<std::string> token;
        boost::split(token, bitmap, boost::is_any_of(" "));

        std::string subModelPath =
            this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/sub_models/";

        std::map<std::string, fs::path> modelSet;
        physics::Model_V models = worldPtr->Models();  // modelptr vector

        bool pauseState = worldPtr->IsPaused();
        worldPtr->SetPaused(true);

        for (physics::ModelPtr model : models)
        {
          std::string modelName = model->GetName();
          if (modelName != "ground_plane" && modelName != "husky")
          {
            // std::cout << "removing " << model->GetName() << std::endl;
            // std::fstream a("/home/onur/2D-lidar-RCnn-Ros/husky_sim/log/log.log", std::ios::out | std::ios::in);
            // a << model->GetName() << std::endl;

            worldPtr->RemoveModel(model);
            // buraya serverden not ready gönderilecek
            // not ready varsa server tekrar reset edecek.

            // model->Reset();
            // model->SetSelected(true);
            // // worldPtr->(model);
            // model->Fini();
            // worldPtr->RemoveModel("removed");
            // gazebo::common::Time::MSleep(500);  // ~model();
          }
        }
        models.clear();

        worldPtr->SetPaused(pauseState);
        worldPtr->ResetEntities(physics::Base::EntityType::MODEL);
        // worldPtr->EnablePhysicsEngine(true);
        worldPtr->EnableAllModels();

        for (const auto& entry : fs::directory_iterator(subModelPath))
        {
          // std::cout << entry << std::endl;
          std::stringstream s;
          s << entry;
          std::string key = s.str();
          key.erase(key.begin());
          key.erase(key.end() - 1);
          // std::cout << key << std::endl;
          if (boost::algorithm::ends_with(key, ".sdf"))
          {
            std::vector<std::string> result;
            boost::split(result, key, boost::is_any_of("/"));
            std::string modelName = result[result.size() - 1].substr(0, result[result.size() - 1].size() - 4);

            // std::cout << " modelName : " << modelName << " key: " << key << std::endl;
            // std::cout << str.at(sequenceWall) << std::endl;
            modelSet.insert({ modelName, key });
          }
        }

        insertWall();

        int i = 0;
        for (auto itr = modelSet.begin(); itr != modelSet.end() && i < token.size(); ++itr, ++i)
        {
          ROS_INFO_STREAM("FACTORY : Stoi" << token[i] << " " << srv.response.bitmapId);
          if (std::stoi(token[i]) == true)
          {
            ROS_INFO_STREAM("FACTORY : Written " << itr->second);
            sdf::SDF model;

            std::ifstream ifSdfStream(itr->second, std::ifstream::in);
            std::stringstream sdfStream;
            std::string sdf;
            sdfStream << ifSdfStream.rdbuf();
            sdf = sdfStream.str();

            model.SetFromString(sdf);
            worldPtr->InsertModelSDF(model);
            ifSdfStream.close();
          }
          else
          {
            continue;
          }
        }
      }
      else
      {
        insertWallWithRemoval();
      }
      // }

      // READY_TO_EXPLORE

      srv.request.in = "READY_TO_EXPLORE";

      if (client.call(srv))
      {
        worldPtr->SetPaused(false);
      }
      // worldPtr->PauseTime();
      // insertModelViaMP(worldPtr);
    }
    worldPtr->ResetTime();
  }

  /// \brief Handle incoming message
  /// \param[in] _msg Repurpose a vector3 message. This function will
  /// only use the x component.

private:
  transport::SubscriberPtr sub;
  physics::WorldPtr worldPtr;
  sdf::ElementPtr sdf;
  std::string buildingEditorPath;
  int modelWallSequence;
  std::string request;
};  // namespace gazebo

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}  // namespace gazebo
