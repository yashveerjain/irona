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
 * @file ironagroup.cpp
 * @author Pavan, Tharun, Yash
 * @brief cpp file for IronaGroup class
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "irona/ironagroup.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>

/**
 * @brief Construct a new Irona Group:: Irona Group object
 *
 */
IronaGroup::IronaGroup()
    : moveBaseClient_("move_base", true),
      cleanRoomServer_(nh_, "cleanroom",
                       boost::bind(&IronaGroup::cleanRoom, this, _1), true) {
  ROS_INFO("Entered the construtor.....");
  detectObjectClient_ = nh_.serviceClient<irona::DetectObject>("detect_object");
  deleteClient =
      nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  // start the action server
  ROS_INFO("CLeanroom service starting....");
  cleanRoomServer_.start();
  ROS_INFO("CLeanroom service done.....");
}

/**
 * @brief Function to trigger the search behavior and detect the object
 *
 * @param goalPose
 *
 * @return true  // if object is found
 * @return false  // if object is not found
 */
bool IronaGroup::searchObject() {
  bool objectFound = false;
  ros::Rate loop_rate(100);
  geometry_msgs::Twist rotate;
  rotate.angular.z = 0.017;
  geometry_msgs::Twist stop;
  double startTime, currentTime;
  for (int i = 0; i < 360; i = i + 20) {
    irona::DetectObject srv;
    // srv.request.a = atoll(argv[1]);
    if (detectObjectClient_.call(srv)) {
      if (srv.response.status) {
        // Store these values in appropriate types
        // srv.response.label;
        objectPose_ = srv.response.pose;
        ROS_INFO("Object found at position x: %f, y: %f, z: %f",
                 objectPose_.position.x, objectPose_.position.y,
                 objectPose_.position.z);
        ROS_INFO("Object found at orientation x: %f, y: %f, z: %f, w: %f",
                 objectPose_.orientation.x, objectPose_.orientation.y,
                 objectPose_.orientation.z, objectPose_.orientation.w);
        objectFound = true;
        return objectFound;
      }
    } else {
      ROS_ERROR("Failed to call service detect object");
    }
    startTime = ros::Time::now().toSec();
    currentTime = startTime;
    while (currentTime - startTime < 20.0) {
      basePublisher_.publish(rotate);
      loop_rate.sleep();
      currentTime = ros::Time::now().toSec();
    }
    for (int j = 0; j < 5; j++) {
      basePublisher_.publish(stop);
      loop_rate.sleep();
    }
  }
  return objectFound;
}

/**
 * @brief Function to obtain the optimal base pose before the arm starts to
 * reach the object
 *
 * @return geometry_msgs::Pose // Optimial pose of the base for the arm to reach
 * the object
 */
geometry_msgs::Pose IronaGroup::getBasePreGraspPose() {
  // Pose relative to object where base
  // should reach before arm starts to
  // reach the object
  geometry_msgs::Pose baseOffsetPose;
  baseOffsetPose.position.x = 0.5;
  baseOffsetPose.orientation.z = 1.0;
  baseOffsetPose.orientation.w = 0.0;
  geometry_msgs::Pose basePreGraspPose;

  // Transform from object to world
  geometry_msgs::TransformStamped objectTransform;
  objectTransform.transform.translation.x = objectPose_.position.x;
  objectTransform.transform.translation.y = objectPose_.position.y;
  objectTransform.transform.translation.z = objectPose_.position.z;
  objectTransform.transform.rotation = objectPose_.orientation;
  tf2::doTransform(baseOffsetPose, basePreGraspPose, objectTransform);
  ROS_INFO("pregrasp position x: %f, y: %f, z: %f", basePreGraspPose.position.x,
           basePreGraspPose.position.y, basePreGraspPose.position.z);
  ROS_INFO("pregrasp orientation x: %f, y: %f, z: %f, w: %f",
           basePreGraspPose.orientation.x, basePreGraspPose.orientation.y,
           basePreGraspPose.orientation.z, basePreGraspPose.orientation.w);
  return basePreGraspPose;
}

/**
 * @brief Function to move the base to a given pose
 *
 * @param basePose  //Target pose for the base
 */
void IronaGroup::moveBase(
    const geometry_msgs::Pose &basePose) {  // ampersand left or right?
  // what if never turns up?
  while (!moveBaseClient_.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose = basePose;
  ROS_INFO("Sending goal");

  moveBaseClient_.sendGoal(goal);
  moveBaseClient_.waitForResult();

  if (moveBaseClient_.getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved");
  } else {
    ROS_INFO("The base failed to move for some reason");
  }
}