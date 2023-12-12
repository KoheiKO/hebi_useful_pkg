#include <ros/ros.h>
#include <ros/console.h> // anable both printf and steram-style output
#include <ros/package.h> // anable to use ros::package command

#include "hebi_cpp_api/lookup.hpp" // anable to find module
#include "hebi_cpp_api/group.hpp" // anable to make a group of module
#include "hebi_cpp_api/group_command.hpp" // anable to send command
#include "hebi_cpp_api/group_feedback.hpp" // anable to get feedback data from module

#include "hebi_useful_pkg/func.hpp"
// /home/kohei/catkin_ws/src/hebi_tutorial/include/hebi_tutorial




void find_module(){
    hebi::Lookup lookup;
    
    // Take snapshot and print screen
    auto entry_list = lookup.getEntryList();
    std::cout << "Modules found on network (Family | Name):" << std::endl;
    for (auto entry : *entry_list){
        std::cout << entry.family_ << " | " << entry.name_ << std::endl;
    }

    std::cout << std::endl
        << " NOTE: " << std::endl
        << "  The listing above should show the information for all the modules" << std::endl
        << "  on the local network.  If this is empty make sure that the modules" << std::endl
        << "  are connected, powered on, and that the status LEDs are displaying" << std::endl
        << "  a green soft-fade." << std::endl;
}



std::shared_ptr<hebi::Group> make_group(std::string FAMILY, std::string MODULE){
    hebi::Lookup lookup;
    auto group = lookup.getGroupFromNames({FAMILY},{MODULE});

    if(!group){
        std::cout << std::endl
            << "Group not found! Check that the family and name of a module on network" << std::endl
            << "matches what is given in the source file" << std::endl;
    }
    else{
        std::cout << "Modules found!" << std::endl;
    }
    std::cout << std::endl
        << "family name | module name" << std::endl
        << FAMILY << " | " << MODULE << "." << std::endl;

    return group;
}  


void find_m3_module(){
    hebi::Lookup lookup;
    
    // Take snapshot and print screen
    auto entry_list = lookup.getEntryList();
    std::cout << "Modules found on network (Family | Name):" << std::endl;
    for (auto entry : *entry_list){
        std::cout << entry.family_ << " | " << entry.name_ << std::endl;
    }

    std::cout << std::endl
        << " NOTE: " << std::endl
        << "  The listing above should show the information for all the modules" << std::endl
        << "  on the local network.  If this is empty make sure that the modules" << std::endl
        << "  are connected, powered on, and that the status LEDs are displaying" << std::endl
        << "  a green soft-fade." << std::endl;
}

std::shared_ptr<hebi::Group> make_m3_group(std::vector<std::string> families, std::vector<std::string> names){
    hebi::Lookup lookup;
    //auto group = lookup.getGroupFromNames(families,names);
    auto group = lookup.getGroupFromNames(families,names);

    if(!group){
        std::cout << std::endl
            << "Group not found! Check that the family and name of a module on network" << std::endl
            << "matches what is given in the source file" << std::endl;
    }
    else{
        std::cout << "Modules found!" << std::endl;
    }
    std::cout << std::endl
     << "family name | module name" << std::endl;
    //  << families << " | " << names[0] << names[1] << "." << std::endl;
    for (const auto& family : families) {
    std::cout << family<< " | ";
    }
    for (const auto& name : names) {
    std::cout << name<< " , ";
    }
    std::cout << "" << std::endl;

    return group;
}  


std::vector<Eigen::Vector2d> readCoordinates() {
    std::vector<Eigen::Vector2d> coordinates;

    coordinates.push_back(Eigen::Vector2d(0, 0));
    coordinates.push_back(Eigen::Vector2d(1, 0));
    // coordinates.push_back(Eigen::Vector2d(1, 0));
    coordinates.push_back(Eigen::Vector2d(1, 1));
    coordinates.push_back(Eigen::Vector2d(0, 1));
    coordinates.push_back(Eigen::Vector2d(0, 0));
    // coordinates.push_back(Eigen::Vector2d(0, 0));

    return coordinates;

}