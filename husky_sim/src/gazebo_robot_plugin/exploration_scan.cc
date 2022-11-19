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
#include <gazebo/physics/physics.hh>
// #include <gazebo/msgs/msgs.hh>

#include <gazebo/common/common.hh>
#include <stdio.h>
#include <cstdlib>
#include <stdarg.h>

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

#define PI 3.14159265359

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

    /* Send notice and error messages to our stdout handler */
    /* Done */
    this->buildingEditorPath = "/home/onur/building_editor_models";
    this->modelWallSequence = 1;
    this->robotPositionSequence = 0;

    reposeRobot();

    std::cout << "intersection : " << IntersectionCheck() << std::endl;
    printf("Random move plugin is creating!\n");
  }

  bool IntersectionCheck()
  {
    initGEOS(geos_message_handler, geos_message_handler);

    /*
        <!-- Base Size -->
  <xacro:property name="base_x_size" value="0.98740000" />
  <xacro:property name="base_y_size" value="0.57090000" />
  <xacro:property name="base_z_size" value="0.24750000" />

    */
    Point point1;
    point1.x = position.x - 0.5;
    point1.y = position.y - 0.3;

    Point point2;
    point2.x = position.x + 0.5;
    point2.y = position.y - 0.3;

    Point point3;
    point3.x = position.x + 0.5;
    point3.y = position.y + 0.3;

    Point point4;
    point4.x = position.x - 0.5;
    point4.y = position.y + 0.3;

    const std::string& robotWKT = PolyrotateUsingYaw(point1, point2, point3, point4, position);

    std::cout << robotWKT << std::endl;
    unsigned char buffer_b[300000];

    std::cout << this->buildingEditorPath + "/wall1/mmap/line_0" << std::endl;
    std::string meshWKBPath = this->buildingEditorPath + "/wall1/mmap/line_0";
    FILE* filp_b = fopen(meshWKBPath.c_str(), "rb");
    int bytes_read_b = fread(buffer_b, sizeof(unsigned char), 300000, filp_b);
    printf("buffer 2 : %d bytes \n", bytes_read_b);

    fclose(filp_b);
    //   unsigned char* a;
    //   a = (unsigned char*)wkb_b;
    //   /* Read the WKT into geometry objects */

    GEOSWKBReader* reader = GEOSWKBReader_create();
    GEOSWKTReader* readerWKT = GEOSWKTReader_create();
    std::cout << "hata 1" << std::endl;
    GEOSGeometry* geom_a = GEOSWKTReader_read(readerWKT, robotWKT.c_str());
    std::cout << "hata 2.5" << std::endl;

    GEOSGeometry* geom_b = GEOSWKBReader_read(reader, buffer_b, bytes_read_b);
    /* Calculate the intersection */
    GEOSGeometry* inter = GEOSIntersection(geom_a, geom_b);
    std::cout << "hata 2" << std::endl;
    /* Convert result to WKT */
    GEOSWKTWriter* writer = GEOSWKTWriter_create();
    /* Trim trailing zeros off output */
    GEOSWKTWriter_setTrim(writer, 1);
    char* wkt_inter = GEOSWKTWriter_write(writer, inter);

    /* Print answer */
    printf("Intersection(A, B): %s\n", wkt_inter);

    /* Clean up everything we allocated */
    GEOSWKTReader_destroy(readerWKT);
    GEOSWKBReader_destroy(reader);
    GEOSWKTWriter_destroy(writer);
    GEOSGeom_destroy(geom_a);
    GEOSGeom_destroy(geom_b);
    GEOSGeom_destroy(inter);
    GEOSFree(wkt_inter);

    /* Clean up the global context */
    finishGEOS();

    return false;
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
    this->simTimePerIter = 10;
    gazebo::common::PoseAnimationPtr anim(
        // name the animation "test",
        // make it last 10 seconds,
        // and set it on a repeat loop
        new gazebo::common::PoseAnimation("test", simTimePerIter, true));
    this->anim = anim;

    explorationLogic(anim);
    this->model->SetAnimation(anim);
    const physics::WorldPtr worldPtr = _parent->GetWorld();
    std::cout << "Animated_box plugin is loaded in " << worldPtr->Name() << std::endl;
    physics::ModelPtr wallModel = worldPtr->ModelByIndex(1);
    std::cout << wallModel->GetName() << std::endl;
  }

  void createRandomPathAnim(gazebo::common::PoseAnimationPtr anim)
  {
    gazebo::common::PoseKeyFrame* key;
    key = anim->CreateKeyFrame(0);
    key->Translation(ignition::math::Vector3d(position.x, position.y, 0.132273));
    key->Rotation(ignition::math::Quaterniond(0, 0, position.yaw));
    // // log frame
    // std::cout << "frame is " << 0 << std::endl;
    // std::cout << "x: " << position.x << "y:" << position.y << "yaw:" << position.yaw << std::endl;

    int sampleFactor = 10;
    int sample = simTimePerIter * sampleFactor;
    for (int i = 1; i <= sample; i++)
    {
      const double keyFrame = i / static_cast<double>(sample) * simTimePerIter;
      key = anim->CreateKeyFrame(keyFrame);
      PositionDerivatives derivatives = createRandomDerivative();
      position.x = position.x + derivatives.dx;
      position.y = position.y + derivatives.dy;
      position.yaw = derivatives.dyaw;
      // // log frane
      // std::cout << "frame is " << keyFrame << std::endl;
      // std::cout << "x: " << position.x << "y:" << position.y << "yaw:" << position.yaw << std::endl;
      key->Translation(ignition::math::Vector3d(position.x, position.y, 0.132273));
      key->Rotation(ignition::math::Quaterniond(0, 0, position.yaw));
    }
  }

  void explorationLogic(gazebo::common::PoseAnimationPtr anim)
  {
    // read random possible robot positions+
    // lms1xx lidar 25hz - 50hz
    PositionDerivatives derivatives = createRandomDerivative();
    position.x = position.x + derivatives.dx;
    position.y = position.y + derivatives.dy;
    position.yaw = derivatives.dyaw;
    // std::cout << "x: " << position.x << "y:" << position.y << "yaw:" << position.yaw << std::endl;

    createRandomPathAnim(anim);
  }
  // yeni robot pozisyonunu dolduracak başa sarılacak
  void reposeRobot()
  {
    std::string robotPositionPath = this->buildingEditorPath + "/wall" + std::to_string(this->modelWallSequence) +
                                    "/robot_position_" + std::to_string(this->robotPositionSequence);
    std::ifstream robotPosFile(robotPositionPath);

    std::string robotPositions;
    // position.x = 6.464248173058339d;
    // position.y = 8.785487837316527d;
    // position.yaw = -2.385220960471571d;

    robotPosFile >> position.x >> position.y >> position.yaw;

    std::cout << "Folder robot position : " << robotPositionPath << std::endl;
    std::cout << std::setprecision(15) << position.x << " " << position.y << " " << position.yaw << std::endl;
    this->robotPositionSequence++;
  }

  void Reset()
  {
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

    this->model->StopAnimation();
    // anim->~PoseAnimation();

    gazebo::common::PoseAnimationPtr anim(
        // name the animation "test",
        // make it last 10 seconds,
        // and set it on a repeat loop
        new gazebo::common::PoseAnimation("test", simTimePerIter, true));
    this->anim = anim;

    explorationLogic(anim);
    this->model->SetAnimation(anim);

    // bu harita eklendiğinde çalışacak
    // const physics::WorldPtr worldPtr = model->GetWorld();
    // std::cout << "Animated_box plugin is loaded in " << worldPtr->Name() << std::endl;
    // physics::ModelPtr wallModel = worldPtr->ModelByIndex(2);
    // std::cout << wallModel->GetName() << std::endl;
  }

private:
  const std::string PolyrotateUsingYaw(Point& point1, Point& point2, Point& point3, Point& point4,
                                       RobotPosition& position)
  {
    double s = sin(position.yaw);
    double c = cos(position.yaw);

    point1.x -= position.x;
    point1.y -= position.y;
    point1.x = point1.x * c - point1.y * s + position.x;
    point1.y = point1.x * s + point1.y * c + +position.y;

    point2.x -= position.x;
    point2.y -= position.y;
    point2.x = point2.x * c - point2.y * s + position.x;
    point2.y = point2.x * s + point2.y * c + position.y;

    point3.x -= position.x;
    point3.y -= position.y;
    point3.x = point3.x * c - point3.y * s + position.x;
    point3.y = point3.x * s + point3.y * c + position.y;

    point4.x -= position.x;
    point4.y -= position.y;
    point4.x = point4.x * c - point4.y * s + position.x;
    point4.y = point4.x * s + point4.y * c + position.y;

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
  int robotPositionSequence;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};
GZ_REGISTER_MODEL_PLUGIN(RobotHello)
}  // namespace gazebo
