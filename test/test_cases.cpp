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
 * @file test_cases.cpp
 * @author Pavan, Tharun, Yash 
 * @brief file for test cases
 *
 * @copyright Copyright (c) 2022
 *
 **/
#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <vector>
#include "irona/ironagroup.hpp"
#include "irona/detector.hpp"

TEST(detector_object, DetectObject) {
    Detector my_detector;
    irona::DetectObject srv;
    irona::DetectObject::Request req;
    irona::DetectObject::Response res;
    srv.request.test = 2;  // just to intitiate service
    EXPECT_EQ(my_detector.objectPoses.size(), 2);

    EXPECT_TRUE(my_detector.detectObject(req, res));
    EXPECT_TRUE(res.status);

    // Test case check if image is empty
    //  subscribing to the camera topic
}

TEST(IronaGroup, getBasePreGraspPose) {
    IronaGroup pirona_test;
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    // Test case for getBasePreGraspPosie
    pirona_test.objectPose_ = pose;
    geometry_msgs::Pose output_pose = pirona_test.getBasePreGraspPose();
    EXPECT_EQ(output_pose.position.x, 0.5);
    EXPECT_EQ(output_pose.position.y, 0.0);
    EXPECT_EQ(output_pose.position.z, 0.0);
    EXPECT_EQ(output_pose.orientation.x, 0.0);
    EXPECT_EQ(output_pose.orientation.y, 0.0);
    EXPECT_EQ(output_pose.orientation.z, 1.0);
    EXPECT_EQ(output_pose.orientation.w, 0.0);
}

// TEST(IronaGroup, searchObject) {
//     IronaGroup pirona_search;
//     // Test case for searchObject
//     // create a client to call the searchObject service
//     EXPECT_TRUE(pirona_search.searchObject());

// }

