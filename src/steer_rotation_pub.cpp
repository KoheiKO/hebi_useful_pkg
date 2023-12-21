#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "hebi_useful_pkg/steer_rot.h"
#include <math.h>
#include <cmath>
#include "geometry_msgs/Point.h"

float x;
float y;
auto goal_x=1;
auto goal_y=1;
double roll;
double pitch;
double yaw;
double distance;
double ini_x;
double ini_y;
double ini_rot_x;
double ini_rot_y;
double ini_rot_z;
double ini_rot_w;
double ini_yaw;
double time_in_seconds;
float coord_x;
float coord_y;

void t_Callback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    // 最初のトランスフォームの情報だけを処理
    if (!msg->transforms.empty()) {
        const auto& transformStamped = msg->transforms[0];
        // ROS_INFO("Received transform message:");
        // ROS_INFO("Translation: x=%f, y=%f, z=%f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        // ROS_INFO("Rotation: x=%f, y=%f, z=%f, w=%f", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        x = transformStamped.transform.translation.x;
        y = transformStamped.transform.translation.y;
        double rot_x = transformStamped.transform.rotation.x;
        double rot_y = transformStamped.transform.rotation.y;
        double rot_z = transformStamped.transform.rotation.z;
        double rot_w = transformStamped.transform.rotation.w;
        double siny_cosp = 2.0 * (rot_w * rot_z + rot_x * rot_y);
        double cosy_cosp = 1.0 - 2.0 * (rot_y * rot_y + rot_z * rot_z);
        yaw = atan2(siny_cosp, cosy_cosp);

        if (time_in_seconds == 1.0){
        ini_x = transformStamped.transform.translation.x;
        ini_y = transformStamped.transform.translation.y;
        ini_rot_x = transformStamped.transform.rotation.x;
        ini_rot_y = transformStamped.transform.rotation.y;
        ini_rot_z = transformStamped.transform.rotation.z;
        ini_rot_w = transformStamped.transform.rotation.w;
        }
        ini_yaw = atan2(2 * (ini_rot_y * ini_rot_z + ini_rot_w * ini_rot_x), ini_rot_w * ini_rot_w - ini_rot_x * ini_rot_x - ini_rot_y * ini_rot_y - ini_rot_z * ini_rot_z);
    }
}

void surveyPointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // ここで受信したメッセージを処理する

    coord_x = msg->x;
    coord_y = msg->y;
    ROS_INFO("Received survey point: x=%f, y=%f, z=%f", msg->x, msg->y, msg->z);
}

int main(int argc, char **argv){
    // ROSノードの初期化
    ros::init(argc, argv, "steer_rotation_pub");
    ros::NodeHandle nh;
    ros::Time current_time = ros::Time::now();
    time_in_seconds = current_time.toSec();
    // トピックのサブスクライバーの作成
    ros::Subscriber sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 100, t_Callback);
    ros::Publisher pub = nh.advertise<hebi_useful_pkg::steer_rot>("/steer_rot",1000);
    ros::Subscriber coord_sub = nh.subscribe("/survey_point", 10, surveyPointCallback);
    ros::Rate loop_rate(100);  // Set the loop rate to 100Hz

    while (ros::ok()) {
        goal_x = coord_x;
        goal_y = coord_y;
        hebi_useful_pkg::steer_rot msg;
        auto delta_yaw = yaw - ini_yaw;
        auto theta = atan2((goal_y - y), (goal_x - x));
        auto steer_rot = -delta_yaw + theta;
        auto st_rot_de = steer_rot * 180 / M_PI;
        msg.x_pos = x;
        msg.y_pos = y;
        msg.steer_rot = steer_rot;
        msg.theta = theta;
        msg.delta_yaw = delta_yaw;
        msg.steer_rot_degree = st_rot_de;
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();  // Sleep to achieve the desired loop rate
    }

    return 0;
}
