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
 * @file talker.cpp
 * @brief A simple talker that publishes to
 *        the topic /chatter
 * @author Banuprathap Anandan
 * @date   03/14/2017
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/ModString.h"
//  string to store the message
//  default initiation to "default msg"
std::string g_str("default msg");

/**
 * @brief      function to handle service
 *
 * @param      req   The request
 * @param      res   The response
 *
 * @return     returns true upon successful execution
 */
bool modify(beginner_tutorials::ModString::Request  &req,
            beginner_tutorials::ModString::Request  &res) {
  ROS_INFO("Service Call initiated");
  g_str = req.s;
  ROS_INFO("Message modification successful");
  return true;
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
  ros::init(argc, argv, "talker");
  /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  /**
   * This creates the service.
   */
  ros::ServiceServer service = n.advertiseService("mod_string", modify);
  //  variable to store loop frequency
  int rate(1);
  // Load parameter
  if (n.hasParam("f")) {
    ROS_INFO("Parameter f available");
    if (n.getParam("f", rate)) {
      ROS_INFO("Rewriting frequency \n");
    }
  }
  ros::Rate loop_rate(rate);
  /**
  * A count of how many messages we have sent. This is used to create
  * a unique string for each message.
  */
  unsigned int count = 0;
  std_msgs::String msg;
  std::stringstream ss;
  while (ros::ok()) {
    ss << g_str << "\t" << count << "\n";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    ss.str("");
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
