#include <ros/ros.h>
#include <ros/console.h> // anable both printf and steram-style output
#include <ros/package.h> // anable to use ros::package command

#include "hebi_cpp_api/lookup.hpp" // anable to find module
#include "hebi_cpp_api/group.hpp" // anable to make a group of module
#include "hebi_cpp_api/group_command.hpp" // anable to send command
#include "hebi_cpp_api/group_feedback.hpp" // anable to get feedback data from module
void find_module();
std::shared_ptr<hebi::Group> make_group(std::string FAMILY, std::string MODULE);


std::vector<Eigen::Vector2d> readCoordinates();