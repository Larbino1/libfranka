#include <Eigen/Dense>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;


struct RegistrationPoints {
    Eigen::MatrixXd points;
    std::vector<std::string> names;
};

json open_json(fs::path filepath);
RegistrationPoints parse_registration_points(json jsondata);
Eigen::MatrixXd parse_trajectory(json jsondata);