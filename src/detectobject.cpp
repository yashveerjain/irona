/**
 * MIT License
 *
 * Copyright (c) 2022 Pavan Mantripragada
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file detectobject.cpp
 * @author Pavan, Tharun, Yash
 * @brief cpp file for detector class
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "irona/detector.hpp"


Detector::Detector() : call_count(0) {
  service = n.advertiseService("detect_object", &Detector::detectObject, this);
  geometry_msgs::Pose objectPose;
  objectPose.position.x = 1.0;
  objectPose.position.y = 0.5;
  objectPose.orientation.z = 1.0;
  objectPose.orientation.w = 0.0;
  objectPoses.push_back(objectPose);
  objectPose.position.x = -1.0;
  objectPose.position.y = -1.0;
  objectPose.orientation.z = 0.0;
  objectPose.orientation.w = 1.0;
  objectPoses.push_back(objectPose);
}

bool Detector::detectObject(irona::DetectObject::Request &req,
                            irona::DetectObject::Response &res) {
  geometry_msgs::Pose objectPose = objectPoses[call_count];
  ROS_INFO("Object found at position x: %f, y: %f, z: %f",
           objectPose.position.x, objectPose.position.y, objectPose.position.z);
  ROS_INFO("Object found at orientation x: %f, y: %f, z: %f, w: %f",
           objectPose.orientation.x, objectPose.orientation.y,
           objectPose.orientation.z, objectPose.orientation.w);
  res.status = true;
  res.pose = objectPose;
  call_count++;
  return true;
}