#include <ros/ros.h>
#include <ros/console.h> // anable both printf and steram-style output
#include <ros/package.h> // anable to use ros::package command

#include "hebi_cpp_api/lookup.hpp" // anable to find module
#include "hebi_cpp_api/group.hpp" // anable to make a group of module
#include "hebi_cpp_api/group_command.hpp" // anable to send command
#include "hebi_cpp_api/group_feedback.hpp" // anable to get feedback data from module
void find_module();
std::shared_ptr<hebi::Group> make_group(std::string FAMILY, std::string MODULE);

void find_m3_module();
// std::shared_ptr<hebi::Group> make_group(std::string FAMILY, std::string MODULE);
// std::shared_ptr<hebi::Group> make_group(std::string FAMILY, std::vector<std::string> module_names);
std::shared_ptr<hebi::Group> make_group(std::vector<std::string> families, std::vector<std::string> names);


std::vector<Eigen::Vector2d> readCoordinates();