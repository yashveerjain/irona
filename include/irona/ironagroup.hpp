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
 * @file ironagroup.hpp
 * @author Pavan, Tharun, Yash
 * @brief hpp file for ironagroup class
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <actionlib/server/simple_action_server.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <ros/ros.h>

#include "irona/CleanRoomAction.h"
#include "irona/DetectObject.h"

#pragma once

/**
 * @brief The IronaGroup class
 *
 */

class IronaGroup {
 public:
  /**
   * @brief Construct a new Irona Group object
   *
   */
  IronaGroup();

  /**
   * @brief Clean room service callback
   *
   * @param goal no. of objects in room
   */
  void cleanRoom(
      const irona::CleanRoomGoalConstPtr &goal);

  /**
   * @brief function to search for objects
   *
   * @return if object is found or not
   */
  bool searchObject();

  /**
   * @brief Function to move the base
   *
   * @param basePose target pose for the base
   */
  void moveBase(
      const geometry_msgs::Pose &basePose); 

  ros::NodeHandle nh_;                     // node handle
  ros::Publisher basePublisher_;           // publisher for base
  ros::ServiceClient detectObjectClient_;  // client for detecting objects
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      moveBaseClient_;  // client for moving the base
  actionlib::SimpleActionServer<irona::CleanRoomAction>
      cleanRoomServer_;                // server for cleaning the room
  irona::CleanRoomFeedback feedback_;  // feedback from the server
  irona::CleanRoomResult result_;      // result from the server
  int objectsFound_ = 0;               // no. of objects to be found
  geometry_msgs::Pose objectPose_;     // pose of the object
  ros::ServiceClient deleteClient;     // client for deleting models
  std::pair<gazebo_msgs::DeleteModelRequest,
            gazebo_msgs::DeleteModelResponse>  // request and response for
                                               // handling gazebo models
                                                   deleteMsg;
};
