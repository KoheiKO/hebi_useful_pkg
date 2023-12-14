#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "hebi_useful_pkg/cur_position.h"
#include <math.h>

float x;
float y;
auto goal_x=1;
auto goal_y=1;
double roll;
double pitch;
double yaw;
double distance;

double Euclid_distance(double x1, double x2, double y1,double y2){
    return sqrt(pow(x2-x1,2)+ pow(y2-y1,2));
}

void t_Callback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    // 最初のトランスフォームの情報だけを処理
    if (!msg->transforms.empty()) {
        const auto& transformStamped = msg->transforms[0];
        //ROS_INFO("Received transform message:");
        //ROS_INFO("Translation: x=%f, y=%f, z=%f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        //ROS_INFO("Rotation: x=%f, y=%f, z=%f, w=%f", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        x = transformStamped.transform.translation.x;
        y = transformStamped.transform.translation.y;
        auto rot_x = transformStamped.transform.rotation.x;
        auto rot_y = transformStamped.transform.rotation.y;
        auto rot_z = transformStamped.transform.rotation.z;
        auto rot_w = transformStamped.transform.rotation.w;

        // 回転角度の計算
        double sinr_cosp = 2.0 * (rot_w * rot_x + rot_y * rot_z);
        double cosr_cosp = 1.0 - 2.0 * (rot_x * rot_x + rot_y * rot_y);
        double roll = atan2(sinr_cosp, cosr_cosp);
        
        double sinp = 2.0 * (rot_w * rot_y - rot_z * rot_x);
        double pitch = asin(sinp);
        
        double siny_cosp = 2.0 * (rot_w * rot_z + rot_x * rot_y);
        double cosy_cosp = 1.0 - 2.0 * (rot_y * rot_y + rot_z * rot_z);
        double yaw = atan2(siny_cosp, cosy_cosp);

        // 距離の計算
        distance = Euclid_distance(goal_x, x, goal_y, y);
        
    }
}


int main(int argc, char **argv) {
    // ROSノードの初期化
    ros::init(argc, argv, "transform_subscriber");
    ros::NodeHandle node2;

    // トピックのサブスクライバーの作成
    ros::Subscriber sub = node2.subscribe<tf2_msgs::TFMessage>("/tf", 100, t_Callback);
    ros::Publisher pub = node2.advertise<hebi_useful_pkg::cur_position>("tf_topic",1000);
    
    // ROSループ
    while (ros::ok()) {
        hebi_useful_pkg::cur_position msg;
        msg.cur_roll = roll;
        msg.cur_pitch = pitch;
        msg.cur_yaw = yaw;
        msg.cur_distance = distance;
        std::cout<< typeid(decltype(roll)).name() << std::endl;
        pub.publish(msg);
        
        msg.cur_roll = roll;
        msg.cur_pitch = pitch;
        msg.cur_yaw = yaw;
        msg.cur_distance = distance;

        pub.publish(msg);

        ROS_INFO("Published: cur_roll=%f, cur_pitch=%f, cur_yaw=%f, cur_distance=%f", roll, pitch, yaw, distance);

        ROS_INFO("Published: cur_roll=%f, cur_pitch=%f, cur_yaw=%f, cur_distance=%f", roll,pitch,yaw,distance);
//     
        ros::spinOnce();  // コールバック関数を呼び出す   
        ros::Rate(1).sleep(); 
    }

    return 0;
}
