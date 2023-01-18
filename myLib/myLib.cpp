#include "myLib.h"

Eigen::Matrix3d skew(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d ret;
  ret << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return ret;
}

Eigen::Matrix<double, 3, 7> offset_jacobian(Eigen::Affine3d transform,
                                            Eigen::Matrix<double, 6, 7> geometric_jacobian,
                                            Eigen::Vector3d offset) {
  Eigen::Matrix<double, 3, 7> translational_jacobian, rotational_jacobian, jacobian;
  translational_jacobian << geometric_jacobian.topRows(3);
  rotational_jacobian << geometric_jacobian.bottomRows(3);
  jacobian << translational_jacobian - skew(transform.rotation() * offset) * rotational_jacobian;
  return jacobian;
}

Eigen::Matrix<double, 3, 7> unit_vec_jacobian(Eigen::Matrix3d R,
                                              Eigen::Matrix<double, 3, 7> Jw,
                                              Eigen::Vector3d u) {
  Eigen::Matrix<double, 3, 7> J_R1 = -skew(R.col(0)) * Jw;
  Eigen::Matrix<double, 3, 7> J_R2 = -skew(R.col(1)) * Jw;
  Eigen::Matrix<double, 3, 7> J_R3 = -skew(R.col(2)) * Jw;
  return J_R1 * u(0) + J_R2 * u(1) + J_R3 * u(2);
}

ImpedanceCoordResult<3, 7> computeWorldCoord(ImpedanceCoordArgs iargs, WorldCoord world){
  Eigen::Vector3d position(iargs.transform * world.offset);
  Eigen::Matrix<double, 3, 1> ee_error(position - world.ref);
  auto ee_jacobian = offset_jacobian(iargs.transform, iargs.J, world.offset);
  
  ImpedanceCoordResult<3, 7> result;
  result.z = ee_error;
  result.dz = ee_jacobian * iargs.dq;
  result.J = ee_jacobian;
  return result;
}

ImpedanceCoordResult<7, 7> computeJointCoord(ImpedanceCoordArgs iargs){
  ImpedanceCoordResult<7, 7> result;
  result.z = iargs.q;
  result.dz = iargs.dq;
  result.J = Eigen::Matrix<double, 7,7>::Identity(7, 7);
  return result;
}

ImpedanceCoordResult<2, 7> computePortCoord(ImpedanceCoordArgs iargs, PortCoord port) {
  Eigen::Vector3d position(iargs.transform * port.offset);
  Eigen::Matrix3d rotation(iargs.transform.rotation());
  Eigen::Vector3d r(position - port.rcm);
  Eigen::Vector3d v1(rotation * port.u1);
  Eigen::Vector3d v2(rotation * port.u2);
  Eigen::Vector2d error;
  error << v1.transpose() * r, v2.transpose() * r;
  Eigen::Matrix<double, 3, 7> translational_jacobian, rotational_jacobian;
  translational_jacobian << offset_jacobian(iargs.transform, iargs.J, port.offset);
  rotational_jacobian << iargs.J.bottomRows(3);
  Eigen::Matrix<double, 3, 7> J_u1 = unit_vec_jacobian(rotation, rotational_jacobian, port.u1);
  Eigen::Matrix<double, 3, 7> J_u2 = unit_vec_jacobian(rotation, rotational_jacobian, port.u2);
  Eigen::Matrix<double, 2, 7> jacobian;
  jacobian << (v1.transpose() * translational_jacobian) + (r.transpose() * J_u1),
      (v2.transpose() * translational_jacobian) + (r.transpose() * J_u2);
  ImpedanceCoordResult<2, 7> ret;
  ret.z = error;
  ret.dz = jacobian * iargs.dq;
  ret.J = jacobian;
  return ret;
}

Eigen::Vector3d register_point(franka::Robot& robot, Eigen::Vector3d offset) {
  franka::RobotState state;
  Eigen::Vector3d pos;
  state = robot.readOnce();
  Eigen::Affine3d transform(Eigen::Map<Eigen::Matrix4d>(state.O_T_EE.data()));
  pos = transform * offset;
  return pos;
}

Eigen::Vector3d register_point_prompt(franka::Robot& robot, Eigen::Vector3d offset) {
  franka::RobotState state;
  Eigen::Vector3d pos;
  char c;
  while (true) {
    while (true) {
      std::cout << "Press enter to register position" << std::endl;

      c = std::getchar();
      if (c == '\n') {
	pos = register_point(robot, offset);
        break;
      }
    }
    std::cout << "Position registered at:\n" << pos << "\nPress enter to confirm";
    c = std::getchar();
    if (c == '\n')
      break;
  }
  return pos;
}

void myLog(const char identifier[8], std::string msg) {
  std::cout << "log:"<< identifier << ":" << msg << "\n";
}

void logTorques(franka::Model& model, franka::RobotState robot_state) {
  auto tau_J = robot_state.tau_J.data();
  auto tau_J_d = robot_state.tau_J_d.data();
  auto gravity = model.gravity(robot_state);
  std::string tau_J_msg("");
  std::string tau_J_d_msg("");
  std::string gravity_msg("");
  for (int i = 0; i < 7; i ++) {
    tau_J_msg = tau_J_msg + std::to_string(tau_J[i]) + ", ";
    tau_J_d_msg = tau_J_d_msg + std::to_string(tau_J_d[i]) + ", ";
    gravity_msg = gravity_msg + std::to_string(gravity[i]) + ", ";
  }
  myLog("tau_J___", tau_J_msg);
  myLog("tau_J_d_", tau_J_d_msg);
  myLog("gravity_", gravity_msg);
}

void logState(franka::RobotState robot_state) {
  auto q = robot_state.q.data();
  auto dq = robot_state.dq.data();
  std::string q_msg("");
  std::string dq_msg("");
  for (int i = 0; i < 7; i ++) {
    q_msg = q_msg + std::to_string(q[i]) + ", ";
    dq_msg = dq_msg + std::to_string(dq[i]) + ", ";
  }
  myLog("q_______", q_msg);
  myLog("dq______", dq_msg);
}

PointMatchResult matchPointSets(Eigen::MatrixXd pointsA, Eigen::MatrixXd pointsB){
  PointMatchResult result = {false, Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.0};
  if (pointsA.rows() != 3 || pointsB.rows() != 3) {
    std::cerr << "pointsA and pointsB must have 3 rows" << std::endl;
    return result;
  }
  if (pointsA.cols() != pointsB.cols()) {
    std::cerr << "pointsA and pointsB must have the same number of columns";
    return result;
  }

  Eigen::Vector3d comA = pointsA.rowwise().mean(), comB = pointsB.rowwise().mean();

  // std::cout << comA << std::endl;
  // std::cout << comB << std::endl;
  Eigen::MatrixXd p_A = pointsA.colwise() - comA, p_B = pointsB.colwise() - comB;

  // std::cout << p_A << std::endl;
  // std::cout << p_B << std::endl;
  Eigen::Matrix3d H = p_A * p_B.transpose();
  // H.fill(0);
  // for (int i = 0; i < NPoints; i++) {
  //   H += p_A.col(i) * p_B.col(i).transpose();
  // }
  // std::cout << H << std::endl;
  
  const int SVDOptions = Eigen::ComputeFullU | Eigen::ComputeFullV;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, SVDOptions);

  svd.computeU();
  svd.computeV();

  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  // std::cout << "Matrix U:" << std::endl << U << std::endl;
  // std::cout << "Matrix V" << std::endl << V << std::endl;

  Eigen::Matrix3d X = V*U.transpose();

  result.rotation = X;
  result.translation = comB - X * comA;

  // std::cout << "Rotation:" << std::endl;
  // std::cout << R << std::endl;
  // std::cout << "Translation:" << std::endl;
  // std::cout << t << std::endl;

  if (abs(X.determinant() - 1) > 1e-6) {
    std::cerr << "Determinant of rotation matrix not equal to 1 (is " <<  X.determinant() << ")... Failed to solve.";
    return result;
  }

  auto err = (result.rotation * pointsA).colwise() + result.translation - pointsB;
  // std::cout << "error" << std::endl << err <<std::endl;
  auto dists = err.array().pow(2).colwise().sum().sqrt();
  result.avg_error = dists.mean();
  result.max_error = dists.maxCoeff();
  result.success = true;
  // std::cout << dists << std::endl;
  // std::cout<< "Average error, max error: " << result.avg_error << ", " << result.max_error << std::endl;
  return result;
}