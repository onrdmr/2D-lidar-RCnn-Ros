#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include "mastering_ros_demo_pkg/demo_srv.h"

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
    modelWallSequence = 1;
    this->buildingEditorPath = "/home/onur/building_editor_models";

    std::cout << "Factory World Plugin Loaded : World " << _parent->Name() << std::endl;
    this->worldPtr = _parent;
    this->sdf = _sdf;

    // Option 3: Insert model from file via message passing.
    insertModelViaMP(_parent);

    // worldPtr->StartTime();
  }

  void insertModelViaMP(physics::WorldPtr worldPtr)
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

    int removedWallSequence = modelWallSequence - 1;
    const std::string removeModel = "walls" + std::to_string(removedWallSequence);
    std::cout << "removedWallSequence: " << removeModel << std::endl;

    worldPtr->RemoveModel(removeModel);
    std::cout << str << " loading." << std::endl;

    msg.set_sdf_filename(str);

    // Pose to initialize the model to
    msgs::Set(msg.mutable_pose(),
              ignition::math::Pose3d(ignition::math::Vector3d(0, 0, 0), ignition::math::Quaterniond(0, 0, 0)));

    // Send the message
    factoryPub->Publish(msg);
    std::cout << "Model with modelWallSequence " << modelWallSequence << " is loaded " << std::endl;

    // sdf::SDF model;

    // std::string sdfPath = this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) +
    //                       "/sub_models/static_cylinder_1.sdf";
    // // std::cout << sdfPath << std::endl;
    // std::ifstream ifSdfStream(sdfPath, std::ifstream::in);
    // std::stringstream sdfStream;
    // std::string sdf;
    // sdfStream << ifSdfStream.rdbuf();

    // sdf = sdfStream.str();

    // std::cout << "dead" << sdf << std::endl;

    // model.SetFromString(sdf);
    // worldPtr->InsertModelSDF(model);

    // ifSdfStream.close();
    modelWallSequence++;
  }

  void Reset()
  {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mastering_ros_demo_pkg::demo_srv>("demo_service");
    mastering_ros_demo_pkg::demo_srv srv;
    std::stringstream ss;
    ss << "SEND_REQ";
    srv.request.in = ss.str();

    if (client.call(srv))
    {
      /*
        int32 wallSequence
        int32 robotPositionId
        int32 bitmapId
        string mapType
        string buildingEditorPath
      */
      ROS_INFO("IN:CLIENT || From Client [%s], Server says [%d] [%d] [%d] [%s] [%s]", srv.request.in.c_str(),
               srv.response.wallSequence, srv.response.robotPositionId, srv.response.bitmapId,
               srv.response.mapType.c_str(), srv.response.buildingEditorPath.c_str());
    }
    std::cout << "world is resetting" << std::endl;
    // worldPtr->PauseTime();
    insertModelViaMP(worldPtr);
  }
  /// \brief Handle incoming message
  /// \param[in] _msg Repurpose a vector3 message. This function will
  /// only use the x component.

private:
  transport::SubscriberPtr sub;
  physics::WorldPtr worldPtr;
  sdf::ElementPtr sdf;
  std::string buildingEditorPath;
  long int modelWallSequence;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}  // namespace gazebo
