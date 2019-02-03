/*
 *MIT License
 *
 *  Copyright (c) 2017 Banuprathap Anandan
 *
 *  AUTHOR : BANUPRATHAP ANANDAN
 *  AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
 *  EMAIL : BPRATHAP@UMD.EDU
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *
 *
 *  Program: Simulator for a rectangular robot
 *
 *
 */

/**
 * @file listener.cpp
 * @brief A simple listener that subscribes to
 *        the topic /chatter
 * @author Banuprathap Anandan
 * @date   03/14/2017
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ModString.h"

/**
 * @brief      Callback function when a message appears
 *             on the topic /chatter
 *
 * @param      msg   The message received from /chatter
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: %s", msg->data.c_str());
}

/**
 * @brief      program entrypoint
 *
 * @param      argc  The argc
 * @param      argv  The argv
 *
 * @return     integer 0 upon exit success \n
 *            integer -1 upon exit failure
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");
  /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;
  /**
  * The subscribe() call is how you tell ROS that you want to receive messages
  * on a given topic.  This invokes a call to the ROS
  * master node, which keeps a registry of who is publishing and who
  * is subscribing.  Messages are passed to a callback function, here
  * called chatterCallback.  subscribe() returns a Subscriber object that you
  * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
  * object go out of scope, this callback will automatically be unsubscribed from
  * this topic.
  *
  * The second parameter to the subscribe() function is the size of the message
  * queue.  If messages are arriving faster than they are being processed, this
  * is the number of messages that will be buffered up before beginning to throw
  * away the oldest ones.
  */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  /**
  * ros::spin() will enter a loop, pumping callbacks.  With this version, all
  * callbacks will be called from within this thread (the main one).  ros::spin()
  * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
  */
  ros::spin();
  return 0;
}

