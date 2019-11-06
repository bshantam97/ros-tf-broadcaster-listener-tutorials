/**
 * MIT License

 Copyright (c) [2019] [Shantam Bajpai]

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

/*Include header files..........................*/
#include <sstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <std_msgs/String.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr &msg) {
  /*
   * Transforbroadcaster: provides an easy way to publish coordinate frame transform information
   */
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  // set the translational component
  transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
  tf::Quaternion q;
  // set the quaternion using fixed axis RPY
  q.setRPY(0, 0, msg->theta);
  // Set the rotational element by quaternion
  transform.setRotation(q);
  /*
   * @param: "world": name of the parent frame
   * @param: name of the child frame
   */
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2) {
    ROS_ERROR("need turtle name as argument");
    return -1;
  }
  turtle_name = argv[1];

  ros::NodeHandle node;
  // Subscribing to a topic and broadcasts an transform as soon as a callback is generated
  ros::Subscriber sub = node.subscribe(turtle_name + "/pose", 10,
                                       &poseCallback);
  ros::spin();
  return 0;
}

