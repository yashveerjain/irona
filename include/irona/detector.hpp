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
 * @file detector.hpp
 * @author Pavan, Tharun, Yash
 * @brief hpp file for detector class
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <ros/ros.h>

#include "irona/DetectObject.h"

/**
 * @brief This class is used to detect objects in the environment
 *
 */
class Detector {
 public:
  /**
   * @brief Construct a new Detector object
   *
   */
  Detector();

  /**
   * @brief This function is used to detect objects in the environment
   *
   * @param req // request from the client to start detection
   * @param res // response to the client
   * @return whether object is detected
   */
  bool detectObject(irona::DetectObject::Request &req,
                    irona::DetectObject::Response &res);

  ros::NodeHandle n;            // node handler
  ros::ServiceServer service;  // service server
  int call_count;                  // number of times the service is called
  std::vector<geometry_msgs::Pose> objectPoses;  // vector of object poses
};