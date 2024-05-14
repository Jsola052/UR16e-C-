#include <iostream>
#include "Auxiliary"
#include "UrRobot.hpp"


void home(UrRobot &robot,double acc, double vel) {
    robot.control().moveJ({-1.57, -1.57, -1.57, -1.57, 1.57, 0}, acc, vel);
}

void getGrinder(UrRobot &robot, double grinder_payload, std::vector<double> &grinder_tcp, std::vector<double> &grinder_cog, int lock, int unlock) {
    home(robot, 0.5, 0.5);
    robot.control().setTcp({0,0,0,0,0,0});
    // tool_changer.write(unlock)
    robot.control().moveL({0.42343, 0.31155, 0.45083, 0, 3.143, 0.000}, 0.3, 0.3);
    robot.control().moveL({0.42343, 0.31155, 0.24508, 0, 3.143, 0.000}, 0.3, 0.3);
    robot.control().moveL({0.42343, 0.31155, 0.22222, 0, 3.143, 0.000}, 0.1, 0.1);  
    // tool_changer.write(lock)
    robot.control().setPayload(grinder_payload, grinder_cog);
    robot.control().moveL({0.42343, 0.31155, 0.24508, 0, 3.143, 0.000}, 0.2, 0.2);
    robot.control().moveL({0.42343, 0.31155, 0.45083, 0, 3.143, 0.000}, 0.3, 0.3);
    home(robot, 0.5, 0.5);
    robot.control().setTcp(grinder_tcp);
}

void returnGrinder(UrRobot &robot, double normal_payload, std::vector<double> &normal_tcp, std::vector<double> &normal_cog, int lock, int unlock) {
    home(robot, 0.5, 0.5);
    robot.control().setTcp(normal_tcp);
    robot.control().moveL({0.42343, 0.31155, 0.45083, 0, 3.143, 0.000}, 0.3, 0.3);
    robot.control().moveL({0.42343, 0.31155, 0.24508, 0, 3.143, 0.000}, 0.3, 0.3);
    robot.control().moveL({0.42343, 0.31155, 0.22222, 0, 3.143, 0.000}, 0.1, 0.1);  
    // tool_changer.write(unlock)
    robot.control().setPayload(normal_payload, normal_cog);
    robot.control().moveL({0.42343, 0.31155, 0.24508, 0, 3.143, 0.000}, 0.2, 0.2);
    robot.control().moveL({0.42343, 0.31155, 0.45083, 0, 3.143, 0.000}, 0.3, 0.3);
    home(robot, 0.5, 0.5);
    robot.control().setTcp(normal_tcp);
}

int main(int argc, char *argv[]) {
    UrRobot robot("172.16.3.114");
    home(robot, 0.5, 0.5);
    int lock = 0;
    int unlock = 1;
    int tool_off = 0;
    int tool_on = 1;
    double grinder_payload = 2.200;
    double normal_payload = 1.100;
    std::vector<double> normal_tcp = {0,0,0,0,0,0};
    std::vector<double> normal_cog = {0,0,0};
    std::vector<double> grinder_tcp = {0.00344, -0.13178, 0.17504, 0.0001, 3.0982, 0.4994};
    std::vector<double> grinder_cog = {-0.008, -0.02, 0.062};
    getGrinder(robot, grinder_payload, grinder_tcp, grinder_cog, lock, unlock);
    Eigen::Vector3d p1 = {-0.192, -0.500, 0.400};
    Eigen::Vector3d p2 = {-0.220, -0.500, 0.400};
    Eigen::Vector3d p3 = {-0.220, -0.400, 0.400};
    Eigen::Vector3d p4 = {-0.192, -0.400, 0.400};
    std::vector<double> current_lin_pos = robot.receive().getActualTCPPose();
    robot.control().moveJ_IK({current_lin_pos[0], current_lin_pos[1], current_lin_pos[2], 0, 0, 0});
    Eigen::Vector3d vector1 = p2 - p1;
    Eigen::Vector3d vector2 = p4 - p1;
    std::vector<double> normal_vector = cross(eigenToVec(vector1), eigenToVec(vector2));
    std::vector<double> eangles = eulerAngles(normal_vector);
    std::vector<double> current_tcp = robot.receive().getActualTCPPose();
    std::vector<double> current_orientation = {current_tcp[3], current_tcp[4], current_tcp[5]};
    Eigen::Vector3d orientation = vecToEigen(eangles) + vecToEigen(current_orientation);
    std::vector<double> new_orientation = eigenToVec(orientation);
    std::vector<Eigen::Vector3d> points = {p1, p2, p3, p4};
    printVector(new_orientation);
    double gridSize = 0.01; 
    double liftDistance = 0.005; 
    double lower = 0.05;
    double rx = orientation[0];
    double ry = orientation[1];
    double rz = orientation[2];
    std::vector<std::vector<double>> waypoints = generateWaypoints(gridSize, liftDistance, lower, rx, ry, rz, points);
    printVector(waypoints);
     // tool.write(tool_on);
    for (const auto &wp : waypoints) 
        robot.control().moveL(wp, 0.2, 0.2);
    // tool.write(tool_off);
    returnGrinder(robot, normal_payload, normal_tcp, normal_cog, lock, unlock);
    home(robot, 0.5, 0.5);
    return 0;
}
