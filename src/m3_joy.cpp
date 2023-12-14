#include <ros/ros.h>
#include <ros/console.h> // anable both printf and steram-style output
#include <ros/package.h> // anable to use ros::package command
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <math.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <fstream> 
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unistd.h> //enable sleep
#include <ctime>

#include <eigen3/Eigen/Dense>
#include "hebi_cpp_api/lookup.hpp" // anable to find module
#include "hebi_cpp_api/group.hpp" // anable to make a group of module
#include "hebi_cpp_api/group_command.hpp" // anable to send command
#include "hebi_cpp_api/group_feedback.hpp" // anable to get feedback data from module
#include "hebi_useful_pkg/joy_info.h"
#include "hebi_useful_pkg/func.hpp"
#include "hebi_useful_pkg/set_name.hpp"

#define PI 3.14159265358979323846
// =============================================================================
//	function
// =============================================================================
void find_m3_module();
std::shared_ptr<hebi::Group> make_m3_group(std::vector<std::string> families, std::vector<std::string> names);
// =============================================================================
//	modules
// =============================================================================
float l_h;
float l_v;
bool cross;
bool circle;
bool triangle;
bool square;
bool l1;
bool r1;

// hebi_intrusion::M3_msg Flag_status;
hebi_useful_pkg::joy_info joy_info;

// void FlagCallback(const hebi_intrusion::M3_msg msg)
// {
//     Flag_status.status = msg.status;
//     ROS_INFO("M3_Received value: %d", Flag_status.status);
// }
void FlagCallback(const hebi_useful_pkg::joy_info msg)
{

	
    joy_info.l_h = msg.l_h;
    joy_info.l_v = msg.l_v;
    joy_info.cross = msg.cross;
    joy_info.circle = msg.circle;
    joy_info.triangle = msg.triangle;
    joy_info.square = msg.square;
    joy_info.l1 = msg.l1;
    joy_info.r1 = msg.r1;

    // ROS_INFO("M3_Received value: %d", Flag_status.status);
}



int main(int argc, char **argv){



    ros::init(argc, argv, "m3_joy"); //reset node name as "hebi_test"
    ros::NodeHandle n; // anable to communiate to ros system


    // ros::Publisher Move_flag = n.advertise<hebi_intrusion::M3_msg>("Move_Flag",1000);

    ros::Subscriber Flag_sub = n.subscribe("/joy_topic", 1000, FlagCallback);



    hebi::Lookup lookup;
    set_name set;
    find_m3_module();
    auto Wheel_group = make_m3_group(set.Wheel_family, set.Wheel_names);
    auto Sus_group = make_m3_group(set.Sus_family, set.Sus_names);
    auto Steer_group = make_m3_group(set.Steer_family, set.Steer_names);



  




    // std::vector<Eigen::Vector2d> coordinates = readCoordinates();

    // double coor_deg_indv[coordinates.size() - 1];

    // std::vector<double> coor_deg ;
    // std::vector<double> move_radian ;
    // double coor_rad[coordinates.size() - 1];
    // double distance[coordinates.size() - 1];
    double wheel_radius = 0.115; // [m]
    // double move_radian_indv[coordinates.size() - 1] ;



    Eigen::VectorXd Wheel_velocities(Wheel_group->size());
    Eigen::VectorXd Wheel_position(Wheel_group->size());
    Eigen::VectorXd wheel_FB_rad(Wheel_group->size());
    Eigen::VectorXd wheel_FB_rad_second(Wheel_group->size());
    // Eigen::MatrixXd wheel_target_positions( Steer_group->size(),coordinates.size() - 1);

    

    hebi::GroupCommand Sus_group_command(Sus_group->size());
    Eigen::VectorXd Sus_velocities(Sus_group->size());
    Eigen::VectorXd Sus_position(Sus_group->size());
    Eigen::VectorXd Sus_position_survey(Sus_group->size());
    Eigen::VectorXd Sus_position_move(Sus_group->size());

    hebi::GroupCommand Steer_group_command(Steer_group->size());
    Eigen::VectorXd Steer_velocities(Steer_group->size());
    Eigen::VectorXd Steer_position_initial_deg(Steer_group->size());
    Eigen::VectorXd Steer_position_rad(Steer_group->size());
    Eigen::VectorXd Steer_position_deg(Steer_group->size());
    Eigen::VectorXd Steer_rotation_rad(Steer_group->size());

    // Eigen::VectorXd Steer_position_initial_pos45(Steer_group->size());

    Eigen::VectorXd Steer_position_initial_rad(Steer_group->size());
    // Eigen::MatrixXd Steer_position_deg( Steer_group->size(),coordinates.size() - 1 );
    // Eigen::MatrixXd Steer_position_rad( Steer_group->size(),coordinates.size() - 1 );

    Eigen::VectorXd steer_FB_rad(Steer_group->size());
    Eigen::VectorXd steer_FB_deg(Steer_group->size());




    // Wheel_velocities << 0.1,0.1,0.1,0.1; // [rad/s] 
    // Wheel_velocities *= 10; 
    Wheel_velocities << 0,0,0,0; // [rad/s] 
    Wheel_velocities *= 10;


    //set Steer Position
    // Steer_position_initial_deg << -90, 0, 90, 180 ; // [degree]
    Steer_position_initial_deg << 0, 0, 0, 0 ; // [degree]


    std::cout << "OK" << std::endl;


    Steer_position_initial_rad = (Steer_position_initial_deg.array() * M_PI / 180.0) ;
    Steer_position_rad = (Steer_position_deg.array() * M_PI / 180.0) ;

    std::cout <<  Steer_position_rad <<std::endl;
    

    hebi::GroupCommand Wheel_group_command(Wheel_group->size());
    Wheel_group_command.setVelocity(Wheel_velocities);

    // Steer_group_command.setVelocity(Steer_velocities);
    Steer_group_command.setPosition(Steer_position_initial_rad);
    // sleep(2);
     // wait for 2 sec

    hebi::GroupFeedback Wheel_feedback(Wheel_group->size());
    hebi::GroupFeedback Steer_feedback(Steer_group->size());
    hebi::GroupFeedback Sus_feedback(Sus_group->size());

 
    Wheel_group->getNextFeedback(Wheel_feedback);
    Eigen::VectorXd wheel_initial_positions = Wheel_feedback.getPosition();





    Wheel_group->setFeedbackFrequencyHz(1000); // set time of feedback
    Steer_group->setFeedbackFrequencyHz(1000);
    Sus_group->setFeedbackFrequencyHz(1000);
 

    // hebi_intrusion::M3_msg pub_msg;
    ros::Rate loop_rate(1000); // send ros command 0.1 Hz(should be shorter than 1/5 Hz)
    // bool Flag = false;

    
            
    while (ros::ok()) {
            ros::spinOnce();    
            loop_rate.sleep();

            Wheel_velocities << joy_info.l_v,joy_info.l_v,joy_info.l_v,joy_info.l_v; // [rad/s] 
            Wheel_velocities *= 10;
            // Steer_position_deg << -90, 0, 90, 180 ; // [degree]
            Steer_rotation_rad << joy_info.l_h ,joy_info.l_h, joy_info.l_h, joy_info.l_h;
            Wheel_velocities *= 0.2;
            Steer_position_rad = (Steer_position_deg.array() * M_PI / 180.0 + Steer_rotation_rad.array()) ;
            hebi::GroupCommand Wheel_group_command(Wheel_group->size());


            Wheel_group_command.setVelocity(Wheel_velocities);
            Wheel_group->sendCommand(Wheel_group_command);

            // Sus_position_move << 0.1, 0.77, 0.29, 0.56;


            // start_time = std::chrono::steady_clock::now();

            //send steer_position to hebi

            Steer_group_command.setPosition(Steer_position_rad);
            Steer_group->sendCommand(Steer_group_command);

            Steer_group->getNextFeedback(Steer_feedback);


    }
    return 0;
}