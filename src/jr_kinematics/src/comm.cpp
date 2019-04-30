#include "ros/ros.h"

#include "hebi_cpp_api/robot_model.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
using BracketType = hebi::robot_model::RobotModel::BracketType;
using LinkType = hebi::robot_model::RobotModel::LinkType;
using namespace hebi;

/*


*/



int main() {
  std::unique_ptr<robot_model::RobotModel> model(new robot_model::RobotModel());
      model->addActuator(ActuatorType::X5_4);
      model->addBracket(BracketType::X5LightLeft);
      model->addActuator(ActuatorType::X8_9);
      model->addLink(LinkType::X5, 0.385064 , M_PI);
      model->addActuator(ActuatorType::X5_9);
      // @TODO Add twist for link below
      model->addLink(LinkType::X5, 0.3302, 0);
      model->addActuator(ActuatorType::X5_1);
      model->addLink(LinkType::X5, 0.101854, 0);
      //model->addActuator(ActuatorType::X5_1);

      Eigen::VectorXd angles(4); 
      angles<<-1.46151,
              1.34995,
              1.00602,
              -0.168123;
      Eigen::Matrix4d transform;
      model->getEndEffector(angles, transform);
      std::cout << "4x4 transform from base to end effector: " << std::endl
          << transform<< std::endl;
} 
