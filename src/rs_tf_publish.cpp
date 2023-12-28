#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <librealsense2/rs.hpp>

void t265_callback(const ros::TimerEvent&, rs2::pipeline& pipeline, tf2_ros::TransformBroadcaster& broadcaster)
{
    // Wait for the next set of frames
    rs2::frameset frames = pipeline.wait_for_frames();

    // Get the T265 pose data
    rs2::pose_frame pose_frame = frames.first_or_default(RS2_STREAM_POSE);
    if (pose_frame)
    {
        rs2_pose pose_data = pose_frame.get_pose_data();

        // Create a TransformStamped message
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "t265_odom_frame";
        transform.child_frame_id = "t265_camera_frame";
        transform.transform.translation.x = pose_data.translation.x;
        transform.transform.translation.y = pose_data.translation.y;
        transform.transform.translation.z = pose_data.translation.z;
        transform.transform.rotation.x = pose_data.rotation.x;
        transform.transform.rotation.y = pose_data.rotation.y;
        transform.transform.rotation.z = pose_data.rotation.z;
        transform.transform.rotation.w = pose_data.rotation.w;

        // Publish the TF transform
        broadcaster.sendTransform(transform);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_t265_tf_publisher");
    ros::NodeHandle nh;

    // Configure the Realsense T265 pipeline
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_POSE);

    // Start the pipeline
    pipeline.start(config);

    // Set up TF broadcaster
    tf2_ros::TransformBroadcaster broadcaster;

    // Set the callback function to run at 10 Hz
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(t265_callback, _1, boost::ref(pipeline), boost::ref(broadcaster)));

    // Spin until the node is shutdown
    ros::spin();

    // Stop the pipeline when the node is shutdown
    pipeline.stop();

    return 0;
}
