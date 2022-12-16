/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <ignition/math/Box.hh>

#include <gazebo/physics/physics.hh>
// #include <gazebo/msgs/msgs.hh>

#include <gazebo/common/common.hh>
#include <stdio.h>
#include <cstdlib>
#include <stdarg.h>
#include <string.h>
/* Only the CAPI header is required */
#include <geos_c.h>
// /* For WKT read/write cpp api unstable warning */
// #include <geos/geom/GeometryFactory.h>
// #include <geos/geom/Geometry.h>
// #include <geos/io/WKBReader.h>
// #include <geos/io/WKBWriter.h>
// using geos::geom::Geometry;
// using geos::geom::GeometryFactory;
// /* WKTReader/WKTWriter */
// using geos::io::WKBReader;
// using geos::io::WKBWriter;

#include <filesystem>

#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <map>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

#define PI 3.14159265359

namespace fs = std::filesystem;

namespace gazebo
{
/*
 * GEOS requires two message handlers to return
 * error and notice message to the calling program.
 *
 *   typedef void(* GEOSMessageHandler) (const char *fmt,...)
 *
 * Here we stub out an example that just prints the
 * messages to stdout.
 */
static void geos_message_handler(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}
typedef struct Position
{
  double x, y, yaw;
} RobotPosition;

typedef struct Point
{
  double x, y;
} Point;
typedef struct PositionDerivatives
{
  double dx, dy, dyaw;
} PositionDerivatives;

class RobotHello : public ModelPlugin
{
public:
  RobotHello() : ModelPlugin()
  {
    srand((unsigned)time(NULL));

    ROBOT_SIZE = 1.2;  // 0.7
    ROBOT_WING = 0.35;

    /* Send notice and error messages to our stdout handler */
    /* Done */
    // this->buildingEditorPath = "/home/onur/building_editor_models";
    // this->modelWallSequence = 1;
    // this->robotPositionSequence = 0;

    // reposeRobot();

    // std::cout << "intersection : " << IntersectionCheck() << std::endl;
    printf("Random move plugin is creating!\n");
  }

  bool IntersectionCheck(double x, double y, double yaw)
  {
    bool intersection = false;
    initGEOS(geos_message_handler, geos_message_handler);

    /*
        <!-- Base Size -->
        <xacro:property name="base_x_size" value="0.98740000" />
        <xacro:property name="base_y_size" value="0.57090000" />
        <xacro:property name="base_z_size" value="0.24750000" />

    */
    Point point1;
    point1.x = x - ROBOT_SIZE;
    point1.y = y - ROBOT_WING;

    Point point2;
    point2.x = x + ROBOT_SIZE;
    point2.y = y - ROBOT_WING;

    Point point3;
    point3.x = x + ROBOT_SIZE;
    point3.y = y + ROBOT_WING;

    Point point4;
    point4.x = x - ROBOT_SIZE;
    point4.y = y + ROBOT_WING;

    RobotPosition centroid;
    centroid.x = x;
    centroid.y = y;
    centroid.yaw = yaw;
    const std::string& robotWKT = PolyrotateUsingYaw(point1, point2, point3, point4, centroid);

    // std::cout << robotWKT << std::endl;
    unsigned char buffer_b[300000];

    GEOSWKTReader* readerWKT = GEOSWKTReader_create();
    GEOSGeometry* geom_a = GEOSWKTReader_read(readerWKT, robotWKT.c_str());
    // std::cout << "multi room intersection check gireceğim " << std::endl;
    if (this->mapType == "multi-room")
    {
      // std::cout << "mmap folder path "
      //           << this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/mmap" <<
      //           std::endl;
      for (const auto& entry : fs::directory_iterator(this->buildingEditorPath + "/wall" +
                                                      std::to_string(this->modelWallSequence) + "/mmap"))
      {
        GEOSWKBReader* reader = GEOSWKBReader_create();
        GEOSWKTWriter* writer = GEOSWKTWriter_create();
        // std::cout << "duvar " << entry << " ve robot arasında kesişim aranıyor." << std::endl;

        std::stringstream meshWKBSS;
        meshWKBSS << entry;
        std::string meshWKBPath = meshWKBSS.str().substr(1, meshWKBSS.str().size() - 2);
        // ROS_INFO_STREAM("EXPLORATION : meshWKBPath : " << meshWKBPath << std::endl);
        FILE* filp_b = fopen(meshWKBPath.c_str(), "rb");
        int bytes_read_b = fread(buffer_b, sizeof(unsigned char), 300000, filp_b);
        // printf("buffer 2 : %d bytes \n", bytes_read_b);
        fclose(filp_b);
        //   unsigned char* a;
        //   a = (unsigned char*)wkb_b;
        //   /* Read the WKT into geometry objects */

        GEOSGeometry* geom_b = GEOSWKBReader_read(reader, buffer_b, bytes_read_b);
        /* Calculate the intersection */
        GEOSGeometry* inter = GEOSIntersection(geom_a, geom_b);

        /* Convert result to WKT */

        /* Trim trailing zeros off output */
        GEOSWKTWriter_setTrim(writer, 1);
        char* wkt_inter = GEOSWKTWriter_write(writer, inter);
        char* wall_geom = GEOSWKTWriter_write(writer, geom_b);

        /* Print answer */
        ROS_INFO("EXPLORATION : Intersection(A, B): %s\n", wkt_inter);
        ROS_INFO("EXPLORATION : WallPolygon(A, B): %s\n", wall_geom);

        char* output = NULL;
        output = strstr(wkt_inter, "EMPTY");

        if (output == NULL)
        {
          intersection = true;
          ROS_INFO("EXPLORATION : NULLDAN KESİŞİM ÇIKTI");
          GEOSWKBReader_destroy(reader);
          GEOSWKTWriter_destroy(writer);
          GEOSGeom_destroy(geom_b);
          GEOSGeom_destroy(inter);
          GEOSFree(wkt_inter);
          GEOSWKTReader_destroy(readerWKT);
          GEOSGeom_destroy(geom_a);
          return intersection;
        }
        GEOSWKBReader_destroy(reader);
        GEOSWKTWriter_destroy(writer);
        GEOSGeom_destroy(geom_b);
        GEOSGeom_destroy(inter);
        GEOSFree(wkt_inter);
      }

      /* Clean up everything we allocated */

      if (this->bitmapId != -1)
      {
        std::map<std::string, fs::path> modelSet;

        for (const auto& entry : fs::directory_iterator(this->buildingEditorPath + "/wall" +
                                                        std::to_string(this->modelWallSequence) + "/sub_models/"))
        {
          // std::cout << entry << std::endl;
          std::stringstream s;
          s << entry;
          std::string key = s.str();
          key.erase(key.begin());
          key.erase(key.end() - 1);
          // std::cout << key << std::endl;
          if (boost::algorithm::ends_with(key, ".wkb"))
          {
            std::vector<std::string> result;
            boost::split(result, key, boost::is_any_of("/"));
            std::string modelName = result[result.size() - 1].substr(0, result[result.size() - 1].size() - 4);

            ROS_INFO_STREAM(" modelName : " << modelName << " key: " << key);

            modelSet.insert({ modelName, key });
          }
        }
        // bitmap
        std::ifstream stream(this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/bitmap_" +
                             std::to_string(this->bitmapId));
        std::stringstream sstream;
        sstream << stream.rdbuf();
        std::string bitmap = sstream.str();
        // std::cout << "bitmap " << bitmap << std::endl;
        std::vector<std::string> token;
        boost::split(token, bitmap, boost::is_any_of(" "));

        int i = 0;

        for (auto itr = modelSet.begin(); itr != modelSet.end() && i < token.size(); ++itr, ++i)
        {
          // std::cout << "stoi" << token[i] << " " << this->bitmapId << std::endl;
          if (std::stoi(token[i]) == true)
          {
            GEOSWKBReader* reader = GEOSWKBReader_create();
            GEOSWKTWriter* writer = GEOSWKTWriter_create();
            ROS_INFO_STREAM("EXPLORATON : duvar " << itr->second << " ve robot arasında kesişim aranıyor.");

            FILE* filp_b = fopen(itr->second.c_str(), "rb");
            int bytes_read_b = fread(buffer_b, sizeof(unsigned char), 300000, filp_b);
            ROS_INFO("EXPLORATION : buffer 2 : %d bytes \n", bytes_read_b);
            fclose(filp_b);
            //   unsigned char* a;
            //   a = (unsigned char*)wkb_b;
            //   /* Read the WKT into geometry objects */

            GEOSGeometry* geom_b = GEOSWKBReader_read(reader, buffer_b, bytes_read_b);
            ROS_INFO("EXPLORATION : read geom_b");
            /* Calculate the intersection */
            GEOSGeometry* inter = GEOSIntersection(geom_a, geom_b);
            ROS_INFO("EXPLORATION : intersection doing..");

            /* Convert result to WKT */

            /* Trim trailing zeros off output */
            GEOSWKTWriter_setTrim(writer, 1);
            char* wkt_inter = GEOSWKTWriter_write(writer, inter);
            char* wall_geom = GEOSWKTWriter_write(writer, geom_b);

            /* Print answer */
            ROS_INFO("EXPLORATION : Intersection(A, B): %s\n", wkt_inter);
            ROS_INFO("EXPLORATION : WallPolygon(A, B): %s\n", wall_geom);

            char* output = NULL;
            output = strstr(wkt_inter, "EMPTY");

            if (output == NULL)
            {
              intersection = true;
              GEOSWKBReader_destroy(reader);
              GEOSWKTWriter_destroy(writer);
              GEOSGeom_destroy(geom_b);
              GEOSGeom_destroy(inter);
              GEOSFree(wkt_inter);
              GEOSWKTReader_destroy(readerWKT);
              GEOSGeom_destroy(geom_a);
              return intersection;
            }
            GEOSWKBReader_destroy(reader);
            GEOSWKTWriter_destroy(writer);
            GEOSGeom_destroy(geom_b);
            GEOSGeom_destroy(inter);
            GEOSFree(wkt_inter);
          }
          else
          {
            continue;
          }

          /* Clean up everything we allocated */
        }
      }
      GEOSWKTReader_destroy(readerWKT);
      GEOSGeom_destroy(geom_a);
    }

    else if (this->mapType == "single-room")
    {
      ROS_INFO_STREAM("EXPLORATION : single-room folder path "
                      << this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/");

      std::string entry = this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/wall.wkb";

      GEOSWKBReader* reader = GEOSWKBReader_create();
      GEOSWKTWriter* writer = GEOSWKTWriter_create();
      ROS_INFO_STREAM("EXPLORATION : duvar " << entry << " ve robot arasında kesişim aranıyor.");

      std::string meshWKBPath = entry;
      // std::cout << "meshWKBPath : " << meshWKBPath << std::endl;
      FILE* filp_b = fopen(meshWKBPath.c_str(), "rb");
      int bytes_read_b = fread(buffer_b, sizeof(unsigned char), 300000, filp_b);
      // printf("buffer 2 : %d bytes \n", bytes_read_b);
      fclose(filp_b);
      //   unsigned char* a;
      //   a = (unsigned char*)wkb_b;
      //   /* Read the WKT into geometry objects */

      GEOSGeometry* geom_b = GEOSWKBReader_read(reader, buffer_b, bytes_read_b);

      // GEOSBuffer(geom_b)

      /* Calculate the intersection */
      bool contain = GEOSContains(geom_b, geom_a);

      /* Convert result to WKT */

      /* Trim trailing zeros off output */
      GEOSWKTWriter_setTrim(writer, 1);
      // char* wkt_inter = GEOSWKTWriter_write(writer, inter);
      char* wall_geom = GEOSWKTWriter_write(writer, geom_b);

      /* Print answer */
      ROS_INFO("EXPLORATION : IsContainFreeSpacePolygon(A, B): %d\n", contain);
      ROS_INFO("EXPLORATION : WallPolygon(A, B): %s\n", wall_geom);

      // char* output = NULL;
      // output = strstr(wkt_inter, "EMPTY");

      if (!contain)
      {
        intersection = true;
        GEOSWKBReader_destroy(reader);
        GEOSWKTWriter_destroy(writer);
        GEOSGeom_destroy(geom_b);
        // GEOSGeom_destroy(inter);
        // GEOSFree(wkt_inter);
        GEOSWKTReader_destroy(readerWKT);
        GEOSGeom_destroy(geom_a);
        return intersection;
      }
      GEOSWKBReader_destroy(reader);
      GEOSWKTWriter_destroy(writer);
      GEOSGeom_destroy(geom_b);
      // GEOSGeom_destroy(inter);
      // GEOSFree(wkt_inter);

      /* Clean up everything we allocated */

      if (this->bitmapId != -1)
      {
        std::map<std::string, fs::path> modelSet;

        for (const auto& entry : fs::directory_iterator(this->buildingEditorPath + "/wall" +
                                                        std::to_string(this->modelWallSequence) + "/sub_models/"))
        {
          // std::cout << entry << std::endl;
          std::stringstream s;
          s << entry;
          std::string key = s.str();
          key.erase(key.begin());
          key.erase(key.end() - 1);

          if (boost::algorithm::ends_with(key, ".wkb"))
          {
            std::vector<std::string> result;
            boost::split(result, key, boost::is_any_of("/"));
            std::string modelName = result[result.size() - 1].substr(0, result[result.size() - 1].size() - 4);

            ROS_INFO_STREAM("EXPLORATION :  modelName : " << modelName << " key: " << key);
            // ROS_INFO_STREAM(str.at(sequenceWall));
            modelSet.insert({ modelName, key });
          }
        }
        // bitmap
        std::ifstream stream(this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) + "/bitmap_" +
                             std::to_string(this->bitmapId));
        std::stringstream sstream;
        sstream << stream.rdbuf();
        std::string bitmap = sstream.str();
        ROS_INFO_STREAM("EXLPORATION : bitmap " << bitmap);
        std::vector<std::string> token;
        boost::split(token, bitmap, boost::is_any_of(" "));

        int i = 0;
        for (auto itr = modelSet.begin(); itr != modelSet.end() && i < token.size(); ++itr, ++i)
        {
          ROS_INFO_STREAM("EXPLORATION : token:" << token[i] << " bitmapId:" << this->bitmapId);
          // std::cout << "stoi" << token[i] << " " << this->bitmapId << std::endl;
          if (std::stoi(token[i]) == true)
          {
            GEOSWKBReader* reader = GEOSWKBReader_create();
            GEOSWKTWriter* writer = GEOSWKTWriter_create();
            ROS_INFO_STREAM("EXPLORATON : duvar " << itr->second << " ve robot arasında kesişim aranıyor.");

            FILE* filp_b = fopen(itr->second.c_str(), "rb");
            int bytes_read_b = fread(buffer_b, sizeof(unsigned char), 300000, filp_b);
            ROS_INFO("EXPLORATION : buffer 2 : %d bytes \n", bytes_read_b);
            fclose(filp_b);
            //   unsigned char* a;
            //   a = (unsigned char*)wkb_b;
            //   /* Read the WKT into geometry objects */

            GEOSGeometry* geom_b = GEOSWKBReader_read(reader, buffer_b, bytes_read_b);
            ROS_INFO("EXPLORATION : read geom_b");
            /* Calculate the intersection */
            GEOSGeometry* inter = GEOSIntersection(geom_a, geom_b);
            ROS_INFO("EXPLORATION : intersection doing..");

            /* Convert result to WKT */

            /* Trim trailing zeros off output */
            GEOSWKTWriter_setTrim(writer, 1);
            char* wkt_inter = GEOSWKTWriter_write(writer, inter);
            char* wall_geom = GEOSWKTWriter_write(writer, geom_b);

            /* Print answer */
            ROS_INFO("EXPLORATION : Intersection(A, B): %s\n", wkt_inter);
            ROS_INFO("EXPLORATION : WallPolygon(A, B): %s\n", wall_geom);

            char* output = NULL;
            output = strstr(wkt_inter, "EMPTY");

            if (output == NULL)
            {
              intersection = true;
              GEOSWKBReader_destroy(reader);
              GEOSWKTWriter_destroy(writer);
              GEOSGeom_destroy(geom_b);
              GEOSGeom_destroy(inter);
              GEOSFree(wkt_inter);
              GEOSWKTReader_destroy(readerWKT);
              GEOSGeom_destroy(geom_a);
              return intersection;
            }
            GEOSWKBReader_destroy(reader);
            GEOSWKTWriter_destroy(writer);
            GEOSGeom_destroy(geom_b);
            GEOSGeom_destroy(inter);
            GEOSFree(wkt_inter);
          }
          else
          {
            continue;
          }

          /* Clean up everything we allocated */
        }
      }
      GEOSWKTReader_destroy(readerWKT);
      GEOSGeom_destroy(geom_a);
    }

    /* Clean up the global context */
    finishGEOS();

    return intersection;
  }

  const PositionDerivatives createRandomDerivative()
  {
    PositionDerivatives derivatives;
    // Get a random number
    derivatives.dx = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 2 - 1;
    derivatives.dy = static_cast<double>(rand()) / static_cast<double>(RAND_MAX) * 2 - 1;
    double yaw = atan2(derivatives.dy, derivatives.dx);  //- PI / 2;
    derivatives.dyaw = yaw;                              //(static_cast<float>(rand() % 314 / 2 - 314 / 2)) / 314;

    // std::cout << "dx: " << derivatives.dx << "dy:" << derivatives.dy << "dyaw:" << derivatives.dyaw << std::endl;

    // Print the random number
    return derivatives;
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _parent;
    this->sdf = _sdf;
    // this->model->GetWorld()
    // create the animation
    this->simTimePerIter = 15;
    // gazebo::common::PoseAnimationPtr anim(
    //     // name the animation "test",
    //     // make it last 10 seconds,
    //     // and set it on a repeat loop
    //     new gazebo::common::PoseAnimation("test", simTimePerIter, false));
    // this->anim = anim;

    ignition::math::AxisAlignedBox rBox = _parent->CollisionBoundingBox();

    ROBOT_SIZE = (rBox.YLength() + 0.1) / 2;  // 0.7
    ROBOT_WING = (rBox.XLength() + 0.1) / 2;  //

    ROS_INFO_STREAM("EXPLORATION : robot size  " << rBox.XLength() << " " << rBox.YLength());

    // exit();
    // explorationLogic(anim);
    // this->model->SetAnimation(anim);
    // const physics::WorldPtr worldPtr = _parent->GetWorld();
    // std::cout << "Animated_box plugin is loaded in " << worldPtr->Name() << std::endl;
    // physics::ModelPtr wallModel = worldPtr->ModelByIndex(1);
    // std::cout << wallModel->GetName() << std::endl;
  }

  void createRandomPathAnim(gazebo::common::PoseAnimationPtr anim)
  {
    gazebo::common::PoseKeyFrame* key;
    key = anim->CreateKeyFrame(0);
    key->Translation(ignition::math::Vector3d(position.x, position.y, 0.132273));
    key->Rotation(ignition::math::Quaterniond(0, 0, position.yaw));
    // // log frame
    ROS_INFO_STREAM("EXPLORATION : frame is " << 0);
    ROS_INFO_STREAM("EXPLORATION : x: " << position.x << "y:" << position.y << "yaw:" << position.yaw);

    int sampleFactor = 2;
    int sample = simTimePerIter * sampleFactor;
    bool map_creator_err = false;
    if (IntersectionCheck(position.x, position.y, position.yaw))
    {
      map_creator_err = true;
      // std::fstream a("/home/onur/2D-lidar-RCnn-Ros/husky_sim/log/log.log", std::ios::out | std::ios::in);
      // a << "ilk durumda kesişim var map_creator_script hatalı" << std::endl;
      ROS_INFO("EXPLORATION : ilk durumda kesişim var map_creator_script hatalı");
      // a.close();
    }
    for (int i = 1; i <= sample; i++)
    {
      // std::cout << "Örnek " << i << " deneniyor." << std::endl;
      PositionDerivatives derivatives = createRandomDerivative();
      double x = position.x + derivatives.dx;
      double y = position.y + derivatives.dy;
      double yaw = derivatives.dyaw;
      // log frame

      if (IntersectionCheck(x, y, yaw))
      {
        ROS_INFO_STREAM("frame " << i << " ci örnekte kesişim bulundu. " << map_creator_err);
        i--;
        continue;
      }
      const double keyFrame = i / static_cast<double>(sample) * simTimePerIter;
      // std::cout << "Örnek " << i << " keyframe" << keyFrame << std::endl;
      key = anim->CreateKeyFrame(keyFrame);

      position.x = x;
      position.y = y;
      position.yaw = yaw;
      ROS_INFO_STREAM("EXPLORATION : frame is " << keyFrame);
      ROS_INFO_STREAM("EXPLORATION : x: " << position.x << "y:" << position.y << "yaw:" << position.yaw);
      key->Translation(ignition::math::Vector3d(position.x, position.y, 0.132273));
      key->Rotation(ignition::math::Quaterniond(0, 0, position.yaw));
    }
  }

  void explorationLogic(gazebo::common::PoseAnimationPtr anim)
  {
    // read random possible robot positions+
    // lms1xx lidar 25hz - 50hz
    // PositionDerivatives derivatives = createRandomDerivative();
    // position.x = position.x + derivatives.dx;
    // position.y = position.y + derivatives.dy;
    // position.yaw = derivatives.dyaw;
    // std::cout << "x: " << position.x << "y:" << position.y << "yaw:" << position.yaw << std::endl;

    createRandomPathAnim(anim);
  }
  // yeni robot pozisyonunu dolduracak başa sarılacak
  void reposeRobot()
  {
    ROS_INFO("EXPLORATION : ROBOT POSITON FILE IS READING");
    std::string robotPositionPath = this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) +
                                    "/robot_position_" + std::to_string(this->robotPositionSequence);
    ROS_INFO_STREAM("EXPLORATION : Folder OF robot position FILE : " << robotPositionPath);
    std::ifstream robotPosFile(robotPositionPath);

    std::string robotPositions;
    // position.x = 6.464248173058339d;
    // position.y = 8.785487837316527d;
    // position.yaw = -2.385220960471571d;

    robotPosFile >> position.x >> position.y >> position.yaw;

    // std::cout << "robot position " << robotPositionPath << " " << position.x << " " << position.y << " " <<
    // position.yaw
    //           << std::endl;
    ROS_INFO_STREAM("EXPLORATION : ROBOT X,Y,yaw info : " << std::setprecision(15) << position.x << " " << position.y
                                                          << " " << position.yaw);
    // this->robotPositionSequence++;
  }

  void Reset()
  {
    // server client ile hangi map olduğu bilgisi alınacak.
    this->model->GetWorld()->ResetTime();
    this->model->GetWorld()->SetPaused(true);
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mastering_ros_demo_pkg::demo_srv>("demo_service");
    mastering_ros_demo_pkg::demo_srv srv;
    std::stringstream ss;
    ss << "SEND_REQ";
    srv.request.in = ss.str();
    ROS_INFO_STREAM("EXPLORATION : Exploratin Scan Requesting: " << ss.str());

    // ros::Duration(1).sleep();
    if (client.call(srv))
    {
      ROS_INFO("EXPLORATION : IN EXLPORATION_SCAN | From Client [%s], Server says [%d] [%d] [%d] [%s] [%s] [%s]",
               srv.request.in.c_str(), srv.response.wallSequence, srv.response.robotPositionId, srv.response.bitmapId,
               srv.response.mapType.c_str(), srv.response.buildingEditorPath.c_str(), srv.response.out.c_str());
      this->buildingEditorPath = srv.response.buildingEditorPath;
      this->modelWallSequence = srv.response.wallSequence;
      this->robotPositionSequence = srv.response.robotPositionId;
      this->mapType = srv.response.mapType;
      this->bitmapId = srv.response.bitmapId;
    }
    if (srv.response.out == "true")
    {
      ROS_INFO("EXPLORATION : EXPLORATION is starting...");
      reposeRobot();
      // change second position
      // position.x = 1;
      // position.y = 1;
      // position.yaw = 2;
      // gazebo::common::PoseAnimationPtr anim(
      //     // name the animation "test",
      //     // make it last 10 seconds,
      //     // and set it on a repeat loop
      //     new gazebo::common::PoseAnimation("test", simTimePerIter, true));
      // this->anim = anim;

      // this->model->StopAnimation();
      // anim->~PoseAnimation();

      gazebo::common::PoseAnimationPtr anim(
          // name the animation "test",
          // make it last 10 seconds,
          // and set it on a repeat loop
          new gazebo::common::PoseAnimation("test", simTimePerIter, false));
      this->anim = anim;

      explorationLogic(anim);
      this->model->SetAnimation(anim);

      // ss.clear();
      // ss.str("");
      // ss << "READY_TO_EXPLORE";
      // srv.request.in = ss.str();
      // ROS_INFO_STREAM("sending request to server " << ss.str());

      // if (client.call(srv))
      // {
      //   this->model->GetWorld()->ResetTime();
      //   ROS_INFO("server response got in EXPLORATION");
      // }
      // bu harita eklendiğinde çalışacak
      // const physics::WorldPtr worldPtr = model->GetWorld();
      // std::cout << "Animated_box plugin is loaded in " << worldPtr->Name() << std::endl;
      // physics::ModelPtr wallModel = worldPtr->ModelByIndex(2);
      // std::cout << wallModel->GetName() << std::endl;
    }
    this->model->GetWorld()->SetPaused(false);
  }

private:
  const std::string PolyrotateUsingYaw(Point& point1, Point& point2, Point& point3, Point& point4,
                                       RobotPosition& position)
  {
    double s = sin(position.yaw);
    double c = cos(position.yaw);

    // std::cout <<"yaw " << position.yaw << std::endl;

    // std::cout << "Önceki ROBOT POLYGON((" + std::to_string(point1.x) + " " + std::to_string(point1.y) + ", " +
    //                  std::to_string(point2.x) + " " + std::to_string(point2.y) + ", " + std::to_string(point3.x) + "
    //                  " + std::to_string(point3.y) + ", " + std::to_string(point4.x) + " " + std::to_string(point4.y)
    //                  + "," + std::to_string(point1.x) + " " + std::to_string(point1.y) + "))"
    //           << std::endl;

    point1.x -= position.x;
    point1.y -= position.y;
    double point1_x = point1.x * c - point1.y * s + position.x;
    double point1_y = point1.x * s + point1.y * c + position.y;
    point1.x = point1_x;
    point1.y = point1_y;

    point2.x -= position.x;
    point2.y -= position.y;
    double point2_x = point2.x * c - point2.y * s + position.x;
    double point2_y = point2.x * s + point2.y * c + position.y;
    point2.x = point2_x;
    point2.y = point2_y;

    point3.x -= position.x;
    point3.y -= position.y;
    double point3_x = point3.x * c - point3.y * s + position.x;
    double point3_y = point3.x * s + point3.y * c + position.y;
    point3.x = point3_x;
    point3.y = point3_y;

    point4.x -= position.x;
    point4.y -= position.y;
    double point4_x = point4.x * c - point4.y * s + position.x;
    double point4_y = point4.x * s + point4.y * c + position.y;
    point4.x = point4_x;
    point4.y = point4_y;

    // std::ofstream deneme("deneme");
    // robot polygon hatalı
    ROS_INFO_STREAM("EXPLORATION : ROBOT POLYGON((" + std::to_string(point1.x) + " " + std::to_string(point1.y) + ", " +
                    std::to_string(point2.x) + " " + std::to_string(point2.y) + ", " + std::to_string(point3.x) + " " +
                    std::to_string(point3.y) + "," + std::to_string(point4.x) + " " + std::to_string(point4.y) + "," +
                    std::to_string(point1.x) + " " + std::to_string(point1.y) + "))");

    return "POLYGON((" + std::to_string(point1.x) + " " + std::to_string(point1.y) + ", " + std::to_string(point2.x) +
           " " + std::to_string(point2.y) + ", " + std::to_string(point3.x) + " " + std::to_string(point3.y) + ", " +
           std::to_string(point4.x) + " " + std::to_string(point4.y) + "," + std::to_string(point1.x) + " " +
           std::to_string(point1.y) + "))";
  }

  // Pointer to the model
private:
  physics::ModelPtr model;
  RobotPosition position;
  sdf::ElementPtr sdf;
  gazebo::common::PoseAnimationPtr anim;
  int simTimePerIter;
  std::string buildingEditorPath;
  int modelWallSequence;
  std::string mapType;
  int bitmapId;
  int robotPositionSequence;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
  double ROBOT_SIZE;  // 0.7
  double ROBOT_WING;
};  // namespace gazebo
GZ_REGISTER_MODEL_PLUGIN(RobotHello)
}  // namespace gazebo
