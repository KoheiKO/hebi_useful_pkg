#include <ros/ros.h>
#include <ros/console.h> // enable both printf and steram-style output
#include <ros/package.h> // enable to use ros::package command
# include "geometry_msgs/Twist.h" // enable to communicate with the topic from leptrino
#include <signal.h>

#include "hebi_cpp_api/lookup.hpp" // enable to find module
#include "hebi_cpp_api/group.hpp" // enable to make a group of module
#include "hebi_cpp_api/group_command.hpp" // enable to send command
#include "hebi_cpp_api/group_feedback.hpp" // enable to get feedback data from module

#include <cmath> //enable to absolute value
#include <unistd.h> //enable sleep
#include "hebi_useful_pkg/func.hpp"
#include "hebi_useful_pkg/set_name.hpp"

#include <limits>

// =============================================================================
//	function
// =============================================================================
void find_module();
std::shared_ptr<hebi::Group> make_group(std::string FAMILY, std::string MODULE);

// =============================================================================
//	modules
// =============================================================================

double vel;




void RQTsterringCallback(const geometry_msgs::Twist& msg){
    
	vel = msg.linear.x; 
} 

// void RQTsterringCallback(const sensor_msgs::Joy& msg){
    
// 	vel = msg.linear.x; 
// } 

int main(int argc, char **argv){
    


	
    ros::init(argc, argv, "hebi_control"); //reset node name as "intrusion"
    ros::NodeHandle nh; // anable to communiate to ros system
    hebi::Lookup lookup;
	set_name set;
    find_module();
    auto group = make_group(set.FAMILY, set.MODULE);
    
	
    /***************************************************************************/
    /** Control the HEBI motor: Send command to the HEBI group & Get feedback **/
    /***************************************************************************/
    
    hebi::GroupCommand group_command(group->size());// control command
    hebi::GroupFeedback feedback(group->size());// Feedback Command
    Eigen::VectorXd velocities(group->size());
    Eigen::VectorXd positions(group->size());

    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 100, RQTsterringCallback); 
	ros::Rate loop_rate(20);


	while(ros::ok()){
		ros::spinOnce(); // run subscriber
		loop_rate.sleep();

		group->getNextFeedback(feedback);
		std::cout << "vel = " << vel << std::endl;
		velocities[0] =  vel; // [rad/s] 
		group_command.setVelocity(velocities);
		group->sendCommand(group_command);
		group->sendFeedbackRequest();

        }   
			
										

}
