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

/*Include header files................................*/

#include <ros/ros.h>
#include <tf/transform_listener.h> // Helps in recieving transforms
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>(
      "turtle2/cmd_vel", 10);
  // Automatically subscribes to ROS transform messages
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()) {
    // Stamped transform gives output like frame id, stamp, child frame id
    tf::StampedTransform transform;
    /**
     * We want the transform from frame 1 to frame 2
     * ros::Time(0): The time at which we want the transform . Providing ros::Time(0) will give
     * latest transform
     * transform: Object in which we store the transform
     * turtle1: Source frame
     * turtle2: Target frame
     */
    try {
      // Get transform between 2 frames
      listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0
        * atan2(transform.getOrigin().y(), transform.getOrigin().x());
    vel_msg.linear.x = 0.5
        * sqrt(
            pow(transform.getOrigin().x(), 2)
                +
               pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
}

