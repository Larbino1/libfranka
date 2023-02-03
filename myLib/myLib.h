#pragma once

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

Eigen::Matrix3d skew(const Eigen::Vector3d& vec);

Eigen::Matrix<double, 3, 7> offset_jacobian(Eigen::Affine3d transform,
                                            Eigen::Matrix<double, 6, 7> geometric_jacobian,
                                            Eigen::Vector3d offset);

Eigen::Matrix<double, 3, 7> unit_vec_jacobian(Eigen::Matrix3d R,
                                              Eigen::Matrix<double, 3, 7> Jw,
                                              Eigen::Vector3d u);

struct PortCoord {
  Eigen::Vector3d u1;
  Eigen::Vector3d u2;
  Eigen::Vector3d rcm;
  Eigen::Vector3d offset;
};

class WorldCoord {
  public:
    Eigen::Vector3d offset;
    Eigen::Vector3d ref;
};

template <int Dim, int NDOF>
struct ImpedanceCoordResult {
  Eigen::Matrix<double, Dim, 1> z;
  Eigen::Matrix<double, Dim, 1> dz;
  Eigen::Matrix<double, Dim, NDOF> J;
};

class ImpedanceCoordArgs {
  public:
    Eigen::Affine3d transform;
    Eigen::Matrix<double, 7, 1> q;  // joint angles
    Eigen::Matrix<double, 7, 1> dq;  // joint velocities
    Eigen::Matrix<double, 6, 7> J;   // geometric_jacobian
    ImpedanceCoordArgs(franka::RobotState robot_state, franka::Model& model, franka::Frame frame)
      : transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data())) //TODO should this be a map??
      , q(robot_state.q.data())
      , dq(robot_state.dq.data())
      , J(model.zeroJacobian(frame, robot_state).data()) 
      {}
};


template <int Dim, int NDOF>
class DiagonalSpringDamper {
 public:
  Eigen::Array<double, Dim, 1> stiffness;
  Eigen::Array<double, Dim, 1> damping;
  Eigen::Matrix<double, Dim, 1> F(ImpedanceCoordResult<Dim, NDOF> coord) {
      Eigen::Matrix<double, Dim, 1> F = (stiffness * coord.z.array() + damping * coord.dz.array());
      return -F;
  };
  Eigen::Matrix<double, NDOF, 1> tau(ImpedanceCoordResult<Dim, NDOF> coord) {
      return coord.J.transpose() * F(coord).matrix();
  };
};

template <int N_breaks>
class PiecewiseSpring {
 public:
  Eigen::Array<double, N_breaks, 1> breakpoints;
  Eigen::Array<double, N_breaks, 1> ys;
  double left_outer_stiffness;
  double right_outer_stiffness;
  double F(double z) {
    double F;
    if (z <= breakpoints(0)){
      F = ys(0) + left_outer_stiffness * (z - breakpoints(0));
    }
    else if (z >= breakpoints(N_breaks-1)) {
      F = ys(N_breaks-1) + right_outer_stiffness * (z - breakpoints(N_breaks-1));
    }
    else {
      int i = 0;
      for (; i < N_breaks-1; i++) {
        if (z >= breakpoints(i) && z <= breakpoints(i+1)) {
          break;
	  }
      }
      F = ys(i) + (ys(i+1) - ys(i)) * (z - breakpoints(i)) / (breakpoints(i+1) - breakpoints(i));
    }
    return -F;
  }

  PiecewiseSpring(Eigen::Array<double, N_breaks, 1> breaks, Eigen::Array<double, N_breaks, 1> forces, double los, double ros) 
    : breakpoints(breaks)
    , ys(forces)
    , left_outer_stiffness(los)
    , right_outer_stiffness(ros)
  {}

  PiecewiseSpring(Eigen::Array<double, N_breaks, 1> breaks, Eigen::Array<double, N_breaks+1, 1> stiffnesses, double F0) {
    double y = 0;
    double c;

    left_outer_stiffness = stiffnesses(0);

    for (int i = 0; i < N_breaks; i++) {	    
      breakpoints(i) = breaks(i);
      ys(i) = y;
      if (breaks(i) < 0 && breaks(i+1) > 0) {
        c = ys(i) + stiffnesses(i+1) * (0 - breaks(i));
      }
      y += stiffnesses(i+1) * (breaks(i+1) - breaks(i));
    }
    right_outer_stiffness = stiffnesses(N_breaks+1);
    ys += F0 - c;
  }
};

ImpedanceCoordResult<2, 7> computePortCoord(ImpedanceCoordArgs iargs, PortCoord port);
ImpedanceCoordResult<7, 7> computeJointCoord(ImpedanceCoordArgs iargs);
ImpedanceCoordResult<3, 7> computeWorldCoord(ImpedanceCoordArgs iargs, WorldCoord world);

Eigen::Vector3d register_point(franka::Robot& robot, Eigen::Vector3d offset);
Eigen::Vector3d register_point_prompt(franka::Robot& robot, Eigen::Vector3d offset);

Eigen::Vector3d register_ee_offset(franka::Robot& robot);

class VirtualPrismaticJoint {
 private:
  franka::Frame frame;
  Eigen::Affine3d A;
  double inertance;
  double damping;

 public:
  double q;
  double qdot;

  VirtualPrismaticJoint(franka::Frame f, Eigen::Affine3d transform, double i, double d)
    : frame(f)
    , A(transform)
    , inertance(i)
    , damping(d)
  {}

  void update(double F, double dt) {
    double qdd = (F - damping*qdot) / inertance;
    q += qdot * dt;
    qdot += qdd * dt;
  }

  ImpedanceCoordResult<3, 8> computeCoord(ImpedanceCoordArgs iargs) {
    // Get prismatic joint axis in world frame
    Eigen::Affine3d T = iargs.transform * A;  // slider -> world frame
    Eigen::Vector3d axis = T.linear() * Eigen::Vector3d({0.0, 0.0, 1.0});  // slider z-axis in world frame

    // Get slider position in world frame
    Eigen::Vector3d o_s ( q * Eigen::Vector3d({0.0, 0.0, 1.0}) ); // slider pos in slider frame
    Eigen::Vector3d o_ee( A * o_s ); // slider pos in end-effector frame
    Eigen::Vector3d z   ( T * o_s ); // slider pos in world-frame

    // Get offset jacobian for slider position
    Eigen::Matrix<double, 3, 7> offset_Jv = offset_jacobian(iargs.transform, iargs.J, o_ee);
    Eigen::Vector3d dz0(offset_Jv * iargs.dq + qdot * axis);

    // Add col 
    Eigen::Matrix<double, 3, 8> J;
    J.leftCols(7) << offset_Jv;
    J.rightCols(1) << axis;

    // extended dq
    Eigen::Matrix<double, 8, 1> dq_ext;
    dq_ext.topRows(7) << iargs.dq;
    dq_ext.bottomRows(1) << qdot;
    Eigen::Matrix<double, 3, 1> dz(J * dq_ext);

    // Return output
    ImpedanceCoordResult<3, 8> slider_coord;
    slider_coord.z = z;
    slider_coord.dz = dz;
    slider_coord.J = J;
    return slider_coord;
  }
};

void myLog(const char identifier[8], std::string msgxyz);

void logTorques(franka::Model& model, franka::RobotState robot_state);
void logState(franka::RobotState robot_state);


template <int Dim>
void logPortError(ImpedanceCoordResult<Dim, 7> portCoord) {
  auto z = portCoord.z;
  std::string port_z_msg("");
  for (int i = 0; i < Dim; i ++) {
    port_z_msg = port_z_msg + std::to_string(z[i]) + ", ";
  }
  myLog("port_err", port_z_msg);
}

struct PointMatchResult {
  bool success;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  double avg_error;
  double max_error;
};

PointMatchResult matchPointSets(Eigen::MatrixXd pointsA, Eigen::MatrixXd pointsB);

bool YesNoPrompt(std::string msg);


char myGetChar();