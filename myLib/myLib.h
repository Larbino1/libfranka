#include <regex>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>

Eigen::Matrix3d skew(const Eigen::Vector3d& vec);

Eigen::Matrix<double, 3, 7> offset_jacobian(Eigen::Affine3d transform, 
                                        Eigen::Matrix<double, 6, 7> geometric_jacobian,
                                        Eigen::Vector3d offset);

Eigen::Matrix<double, 3, 7> unit_vec_jacobian(Eigen::Matrix3d R, Eigen::Matrix<double, 3, 7> Jw, Eigen::Vector3d u);

struct PortCoord{
  Eigen::Vector3d u1;
  Eigen::Vector3d u2;
  Eigen::Vector3d rcm; 
  Eigen::Vector3d offset;
};


template <int Dim>
struct CoordResult {
    Eigen::Matrix<double, Dim, 1> error;
    Eigen::Matrix<double, Dim, 7> jacobian;
};

CoordResult<2> computePortCoord(Eigen::Affine3d transform, Eigen::Matrix<double, 6, 7> geometric_jacobian, PortCoord port);

struct DRef{
  Eigen::Vector3d r;
  Eigen::Vector3d dr;
};

const std::string FLOAT = "([+-]?[0-9]*[.]?[0-9]+)";
const std::regex REF("x" + FLOAT + " y" + FLOAT + " z" + FLOAT + " dx" + FLOAT + " dy" + FLOAT + " dz" + FLOAT);
DRef read_reference_input();
