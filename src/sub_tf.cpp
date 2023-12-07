#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"

float x;
float y;

void transformCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    // トピックから受信したTFMessageメッセージのデータを処理
    for (const auto& transformStamped : msg->transforms) {
        ROS_INFO("Received transform message:");
        ROS_INFO("Translation: x=%f, y=%f, z=%f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        ROS_INFO("Rotation: x=%f, y=%f, z=%f, w=%f", transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        x = transformStamped.transform.translation.x;
        y = transformStamped.transform.translation.y;
    }
}

// void transformCallback(const tf2_msgs::TFMessage& msg) {
//     // トピックから受信したTransformStampedメッセージのデータを処理
//     ROS_INFO("Received transform message:");
//     ROS_INFO("Translation: x=%f, y=%f, z=%f", msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z);
//     ROS_INFO("Rotation: x=%f, y=%f, z=%f, w=%f", msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w);
//     x = msg.transform.translation.x;
//     y = msg.transform.translation.y;
// }

int main(int argc, char **argv) {
    // ROSノードの初期化
    ros::init(argc, argv, "transform_subscriber");
    ros::NodeHandle nh;

    // トピックのサブスクライバーの作成
    ros::Subscriber sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 100, transformCallback);

    // ROSループ
    while (ros::ok()) {
        ros::spinOnce();  // コールバック関数を呼び出す
        // ここに必要な処理を追加

        // 例: 特定の条件でノードを終了する場合
        // if (some_condition) {
        //     break;
        // }
        
        ros::Rate(10).sleep();  // 10Hzでスリープ
    }

    return 0;
}
