#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "nlohmann/json.hpp"
#include "lie_group/so2.h"
#include "lie_group/se2.h"


using json = nlohmann::json;


int main() {

  std::string diff_drive_params = "diffdrive.json";

  std::ifstream file(diff_drive_params);
  if (!file.is_open()) {
      std::cerr << "Failed to open file: " << std::endl;
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

}