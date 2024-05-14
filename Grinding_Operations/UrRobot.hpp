#ifndef __URROBOT_HPP__
#define __URROBOT_HPP__

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <string>

class UrRobot {
public:
    UrRobot(std::string ip);
    ur_rtde::RTDEReceiveInterface &receive() { return m_receive; }
    ur_rtde::RTDEControlInterface &control() { return m_control; }
private:
    ur_rtde::RTDEReceiveInterface m_receive;
    ur_rtde::RTDEControlInterface m_control;
};

#endif