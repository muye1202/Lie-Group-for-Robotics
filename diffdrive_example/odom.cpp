#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "nlohmann/json.hpp"
#include "so2.h"
#include "se2.h"


using json = nlohmann::json;


void saveToJsonFile(const std::string& filename, 
                    const std::vector<std::pair<double, double>>& vectors) {
  nlohmann::json jsonData = vectors;
  std::ofstream outFile(filename);
  outFile << jsonData.dump(4); // Pretty print with an indent of 4 spaces
  outFile.close();
}


int main() {

  std::string workspace = "/home/muye/repos/lie_group_lib/";

  // Set the wheel radius and track width of the robot
  std::string diff_drive_params = workspace + "diffdrive_example/diffdrive.json";
  std::ifstream file(diff_drive_params);
  if (!file.is_open()) {
      std::cerr << "Failed to open file" << std::endl;
      return 1;
  }

  try {
      json params;
      file >> params;

      double wheel_radius, track_width;
      if (params.contains("wheel_radius") && 
          params.contains("track_width")) {
          wheel_radius = params["wheel_radius"];
          track_width = params["track_width"];
      }

  } catch (const std::exception& e) {
      std::cerr << "Error parsing JSON: " << e.what() << std::endl;
      return 1;
  }

  // Let the robot goes in a circle
  Eigen::Vector3d body_twist(0.628, 0.0, 0.628);
  std::cout << "body twist components\n:";
  std::cout << "vx: " << body_twist(0) << " vy: " << body_twist(1) << " vz: " << body_twist(2) << std::endl;

  // Let the robot run for 10 sec with this twist
  std::vector<std::pair<double, double>> robot_positions;
  SE2::Pose T;
  double total_step = 500, interval = 10./500;

  robot_positions.push_back(T.GetTransPair());
  for (int i=0; i < int(total_step); ++i) {
    // transformation from t to t+1
    // the time interval is 1 sec
    Eigen::Matrix3d del_T = SE2::ExpMap(body_twist*interval);
    SE2::Pose T_prime(del_T);
    T = T * T_prime;

    std::pair<double, double> rob_pos = T.GetTransPair();
    std::cout << "current robot pos " << rob_pos.first << " " << rob_pos.second << std::endl;

    robot_positions.push_back(T.GetTransPair());
  }

  // save the positions to json file
  saveToJsonFile(workspace + "diffdrive_example/robot_pos.json", robot_positions);
}