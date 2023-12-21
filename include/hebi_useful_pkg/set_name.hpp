#include <ros/ros.h>

#ifndef C_HPP
#define C_HPP
class set_name{
public:
    // std::string FAMILY = {"HEBI"}; //module family name

    // std::string MODULE  = {"Intrusion"}; // module name in the group

    std::string FAMILY = {"Snap-through"}; //module family name
    
    std::string MODULE  = {"Left"}; // module name in the group
    

    // std::vector<std::string> names {"module-1","module-2","",""};
    // std::vector<std::string> Wheel_family {"Wheel01"}; //module family name
    // std::vector<std::string> Sus_family {"Sus01"}; //module family name
    // std::vector<std::string> Steer_family {"Steer01"}; //module family name

    std::vector<std::string> Wheel_family {"Wheel02"}; //module family name
    std::vector<std::string> Sus_family {"Sus02"}; //module family name
    std::vector<std::string> Steer_family {"Steer02"}; //module family name

    std::vector<std::string> Wheel_names {"Wheel-1","Wheel-2","Wheel-3","Wheel-4"};
    std::vector<std::string> Steer_names {"Steer-1","Steer-2","Steer-3","Steer-4"};  
    std::vector<std::string> Sus_names {"Sus-1","Sus-2","Sus-3","Sus-4"};
};
#endif // C_HPP