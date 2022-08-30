#include "myLib.h"

Eigen::Matrix3d skew(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d ret; ret << 0,     -vec(2), vec(1),
                              vec(2), 0,     -vec(0),
                             -vec(1), vec(0), 0;
  return ret;
}

Eigen::Matrix<double, 3, 7> offset_jacobian(Eigen::Affine3d transform, Eigen::Matrix<double, 6, 7> geometric_jacobian, Eigen::Vector3d offset) {
      Eigen::Matrix<double, 3, 7> translational_jacobian, rotational_jacobian, jacobian;
      translational_jacobian << geometric_jacobian.topRows(3);
      rotational_jacobian << geometric_jacobian.bottomRows(3);
      jacobian << translational_jacobian - skew(transform.rotation() * offset) * rotational_jacobian;
      return jacobian;
}

Eigen::Matrix<double, 3, 7> unit_vec_jacobian(Eigen::Matrix3d R, Eigen::Matrix<double, 3, 7> Jw, Eigen::Vector3d u) {
  Eigen::Matrix<double, 3, 7> J_R1 = - skew(R.col(0)) * Jw;
  Eigen::Matrix<double, 3, 7> J_R2 = - skew(R.col(1)) * Jw;
  Eigen::Matrix<double, 3, 7> J_R3 = - skew(R.col(2)) * Jw;
  return J_R1*u(0) + J_R2*u(1) + J_R3*u(2);
}

CoordResult<2> computePortCoord(Eigen::Affine3d transform, Eigen::Matrix<double, 6, 7> geometric_jacobian, PortCoord port) {
      Eigen::Vector3d position(transform * port.offset);
      Eigen::Matrix3d rotation(transform.rotation());
      Eigen::Vector3d r(position - port.rcm);
      Eigen::Vector3d v1(rotation * port.u1);
      Eigen::Vector3d v2(rotation * port.u2);
      Eigen::Vector2d error; error << v1.transpose() * r,
                                      v2.transpose() * r;
      Eigen::Matrix<double, 3, 7> translational_jacobian, rotational_jacobian;
      translational_jacobian << offset_jacobian(transform, geometric_jacobian, port.offset);
      rotational_jacobian << geometric_jacobian.bottomRows(3);
      Eigen::Matrix<double, 3, 7> J_u1 = unit_vec_jacobian(rotation, rotational_jacobian, port.u1);
      Eigen::Matrix<double, 3, 7> J_u2 = unit_vec_jacobian(rotation, rotational_jacobian, port.u2);
      Eigen::Matrix<double, 2, 7> jacobian; 
      jacobian << (v1.transpose() * translational_jacobian) + (r.transpose() * J_u1),
                  (v2.transpose() * translational_jacobian) + (r.transpose() * J_u2);
      CoordResult<2> ret;
      ret.error = error;
      ret.jacobian = jacobian;
      return ret;
}

std::string stringIn, line;

DRef read_reference_input() { 
  //std::cin >> stringIn;
  //std::stringstream ss(stringIn); 
  std::getline(std::cin, line);
  
  std::smatch m;
  if (std::regex_search(line, m, REF)){
    stringIn.clear(); 
   /* std::cout << "\n" << m[0] << "\n";
    std::cout << m[1] << "\n";
    std::cout << m[2] << "\n";
    std::cout << m[3] << "\n";
    std::cout << m[4] << "\n";
    std::cout << m[5] << "\n";
    std::cout << m[6] << "\n";*/

    // TODO check error not too large
    DRef ref;
    ref.r << std::stod(m[1]), std::stod(m[2]), std::stod(m[3]);
    ref.dr << std::stod(m[4]), std::stod(m[5]), std::stod(m[6]); 
    return ref;
  }
}
