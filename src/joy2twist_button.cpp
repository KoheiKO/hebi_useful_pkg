#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TwistPublisher{
public:
  TwistPublisher() : nh_(), pnh_("~") {
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("pub_button", 1);
    joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPublisher::timerCallback, this);
  }

  void joyCallback(const sensor_msgs::Joy& joy_msg) {
    last_joy_ = joy_msg;
  }

  void timerCallback(const ros::TimerEvent& e) {
    int assign_circle = 2;
    int assign_triangle = 3;
    int assign_square = 0;
    int assign_cross = 1;
    int assign_L1 = 4;
    int assign_R1 = 5;

    pnh_.getParam("assign_circle", assign_circle);
    pnh_.getParam("assign_triangle", assign_triangle);
    pnh_.getParam("assign_square", assign_square);
    pnh_.getParam("assign_cross", assign_cross);
    pnh_.getParam("assign_L1", assign_L1);
    pnh_.getParam("assign_R1", assign_R1);




    geometry_msgs::Twist cmd_vel;
    if(0 < assign_circle){
      cmd_vel.linear.x = last_joy_.buttons[assign_circle];
    }
    if(0 < assign_triangle){
      cmd_vel.linear.y = last_joy_.buttons[assign_triangle];
    }
    if(0 < assign_square){
      cmd_vel.linear.z = last_joy_.buttons[assign_square];
    }
    if(0 < assign_cross){
      cmd_vel.angular.x = last_joy_.buttons[assign_cross];
    }
    if(0 < assign_L1){
      cmd_vel.angular.y = last_joy_.buttons[assign_L1];
    }
    if(0 < assign_R1){
      cmd_vel.angular.z = last_joy_.buttons[assign_R1];
    }

    cmd_pub_.publish(cmd_vel);

  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer_;
  sensor_msgs::Joy last_joy_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy2twist_button");
  TwistPublisher twist_publisher;
  ros::spin();
  return 0;
}
