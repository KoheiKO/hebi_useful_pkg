#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <hebi_useful_pkg/joy_info.h>
// #include <hebi_useful_pkg/joy_info.h>
float l_h;
float l_v;
bool cross;
bool circle;
bool triangle;
bool square;
bool l1;
bool r1;
bool dir_up;
bool dir_down;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
  // 処理内容を記述

  ROS_INFO("L stick horizontal:%f", joy_msg.axes[0]);   
  ROS_INFO("L stick vertical:%f", joy_msg.axes[1]);
  ROS_INFO("cross:%d", joy_msg.buttons[0]);   
  ROS_INFO("circle:%d", joy_msg.buttons[1]);
  ROS_INFO("triangle:%d", joy_msg.buttons[2]); 
  ROS_INFO("square:%d", joy_msg.buttons[3]);   
  ROS_INFO("l1:%d", joy_msg.buttons[4]);
  ROS_INFO("r1:%d", joy_msg.buttons[5]);  
  ROS_INFO("dir_up:%d", joy_msg.buttons[13]);
  ROS_INFO("dir_down:%d", joy_msg.buttons[14]); 
  l_h = joy_msg.axes[0];  
  l_v = joy_msg.axes[1];
  cross = joy_msg.buttons[0];   
  circle = joy_msg.buttons[1];
  triangle = joy_msg.buttons[2]; 
  square = joy_msg.buttons[3];   
  l1 = joy_msg.buttons[4];
  r1 = joy_msg.buttons[5];
  dir_up = joy_msg.buttons[13];
  dir_down = joy_msg.buttons[14];  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_pubsub_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joy", 1000, joy_callback);
  ros::Publisher pub = nh.advertise<hebi_useful_pkg::joy_info>("joy_topic",1000);


  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    hebi_useful_pkg::joy_info msg;
    msg.l_h = l_h;
    msg.l_v = l_v;
    msg.cross = cross;
    msg.circle = circle;
    msg.triangle = triangle;
    msg.square = square;
    msg.l1 = l1;
    msg.r1 = r1;
    msg.dir_up = dir_up;
    msg.dir_down = dir_down;
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
