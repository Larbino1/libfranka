#include "myJson.h"

json open_json(fs::path filepath) {
  std::ifstream f;
  json ret({}); // Returns null json if fails
  f.open(filepath);
  if (f.is_open()) {
    if (f.peek() == std::ifstream::traits_type::eof()) {
      std::cerr << "Specified file " << filepath << " appears to be empty.\n";
    }
    else {
      ret = json::parse(f);
    }
  } else {
      std::cerr << "Could not open file " << filepath << ". " << strerror(errno) << std::endl;
  }
  return ret;
}


RegistrationPoints parse_registration_points(json jsondata) {
  json points = jsondata["points"];
  int NPoints = points.size();
  Eigen::MatrixXd pointsA(3, NPoints);
  std::vector<std::string> names;
  int i = 0;
  for (auto it = points.begin(); it != points.end(); it++) {
      json point = it.value();
      json data = point["data"];
      Eigen::Vector3d p_A(data[0], data[1], data[2]);
      pointsA.col(i) << p_A;
      names.push_back(point["name"]);
      i += 1;
  }

  RegistrationPoints ret;
  ret.points = pointsA;
  ret.names = names;
  return ret;
}

void save_frame_transform(fs::path filepath, Eigen::Affine3d tf) {
  std::ofstream f(filepath);
  bool failed = false;
  if (f.is_open()) {
    auto jsonf = json{
      {"transform", json::array()}
    };
    Eigen::Matrix4d M;
    M = tf.matrix();
    for (int i = 0; i <= 3; i++) {
        auto& outer = jsonf["transform"];
        for (int j = 0; j <= 3; j++) {
            // You'll need to convert whatever Items is into a JSON type first
            outer[i].push_back(M(i, j));
        }
    }
    f << jsonf;
  } else {
      std::cout << "Could not open file " << filepath << ". " << strerror(errno) << std::endl;
      failed = true;
  }
  if (failed) {
    exit(0);
  }
}

Eigen::Affine3d parse_frame_transform(json jsondata){
  json trajectory = jsondata["transform"];
  Eigen::Affine3d ret;
  int i = 0;
  for (auto it = trajectory.begin(); it != trajectory.end(); it++) {
      json row = it.value();
      std::cout << row << std::endl;
      ret.matrix()(i, 0) = row[0];
      ret.matrix()(i, 1) = row[1];
      ret.matrix()(i, 2) = row[2];
      ret.matrix()(i, 3) = row[3];
      i++;
  }
  return ret;
}

// union PointsOrTransform {
//     RegistrationPoints points;
//     Eigen::Affine3d transform;
// };

PointsOrTransform load_points_or_cached_transform(fs::path points_path, fs::path tf_path) {
  json points_json = open_json(points_path);
  json tf_json = open_json(tf_path);

  
  PointsOrTransform ret;
  ret.points = parse_registration_points(points_json);
    if (!tf_json.is_null()) {
    if (YesNoPrompt("Use cached transform? [y/n]")) {
      ret.got_transform = true;
      ret.transform = parse_frame_transform(tf_json);
    }
  }
  return ret;
}



Eigen::MatrixXd parse_trajectory(json jsondata) {
  json trajectory = jsondata["trajectory"];
  int NPoints = trajectory.size();
  Eigen::MatrixXd ret(3, NPoints);
  std::vector<std::string> names;
  int i = 0;
  for (auto it = trajectory.begin(); it != trajectory.end(); it++) {
      json point = it.value();
      Eigen::Vector3d p_A(point[0], point[1], point[2]);
      ret.col(i) << p_A;
      i += 1;
  }
  return ret;
}