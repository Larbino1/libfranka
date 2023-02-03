#include "myLib.h"
#include <Eigen/Dense>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

json open_json(fs::path filepath);

struct RegistrationPoints {
    Eigen::MatrixXd points;
    std::vector<std::string> names;
};

RegistrationPoints parse_registration_points(json jsondata);

void save_frame_transform(fs::path filepath, Eigen::Affine3d tf);

Eigen::Affine3d parse_frame_transform(json jsondata);

struct PointsOrTransform {
    bool got_transform;
    RegistrationPoints points;
    Eigen::Affine3d transform;
};

PointsOrTransform load_points_or_cached_transform(fs::path registration_points, fs::path transform_cache);


Eigen::MatrixXd parse_trajectory(json jsondata);