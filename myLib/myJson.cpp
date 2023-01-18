#include "myJson.h"

json open_json(fs::path filepath) {
  const char *homedir;
  homedir = getenv("HOME");
  fs::path home(homedir);
  filepath = home / filepath;
  std::ifstream f;
  f.open(filepath);
  bool failed = false;
  if (f.is_open()) {
    if (f.peek() == std::ifstream::traits_type::eof()) {
      std::cerr << "Specified file " << filepath << " appears to be empty.\n";
      failed = true;
    }
    else {
      json data = json::parse(f);
      return data;
    }
  } else {
      std::cout << "Could not open file " << filepath << ". " << strerror(errno) << std::endl;
      failed = true;
  }
  if (failed) {
    exit(0);
  }
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