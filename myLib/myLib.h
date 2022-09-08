#include <iostream>

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
    Eigen::Matrix<double, 7, 1> dq;  // joint velocities
    Eigen::Matrix<double, 6, 7> J;   // geometric_jacobian
    //Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq;  // joint velocities
    //Eigen::Map<const Eigen::Matrix<double, 6, 7>> J;   // geometric_jacobian
    ImpedanceCoordArgs(franka::RobotState robot_state, franka::Model& model, franka::Frame frame)
      : transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()))
      , dq(robot_state.dq.data())
      , J(model.zeroJacobian(frame, robot_state).data()) 
      {}
};


template <int Dim, int NDOF>
class DiagonalSpringDamper {
 public:
  Eigen::Array<double, Dim, 1> stiffness;
  Eigen::Array<double, Dim, 1> damping;
  Eigen::Matrix<double, NDOF, 1> F(ImpedanceCoordResult<Dim, NDOF> coord) {
      return coord.J.transpose() * (-stiffness * coord.z.array() - damping * coord.dz.array()).matrix();
  }
};

ImpedanceCoordResult<2, 7> computePortCoord(ImpedanceCoordArgs iargs, PortCoord port);
ImpedanceCoordResult<3, 7> computeWorldCoord(ImpedanceCoordArgs iargs, WorldCoord world);

Eigen::Vector3d register_point(franka::Robot& robot, Eigen::Vector3d offset);
Eigen::Vector3d register_point_prompt(franka::Robot& robot, Eigen::Vector3d offset);

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
    Eigen::Affine3d T = iargs.transform * A;
    Eigen::Vector3d axis = T.linear() * Eigen::Vector3d({0.0, 0.0, 1.0});

    // Get slider position in world frame
    Eigen::Vector3d offset = A*(q * axis); //position of slider in ee_frame
    Eigen::Vector3d z(iargs.transform * A * (q * axis)); // position of slider in world frame

    // Get offset jacobian for slider position
    Eigen::Matrix<double, 3, 7> offset_Jv = offset_jacobian(iargs.transform, iargs.J, offset);

    // Add col 
    Eigen::Matrix<double, 3, 8> J;
    J.leftCols(7) << offset_Jv;
    J.rightCols(1) << axis;

    // extended dq
    Eigen::Matrix<double, 8, 1> dq_ext;
    dq_ext.topRows(7) << iargs.dq;
    dq_ext.bottomRows(1) << qdot;
    Eigen::Matrix<double, 3, 1> dz(J * dq_ext);

    //std::cout << "J_geo=\n" << iargs.J << "\n";
    //std::cout << "J_o=\n" << offset_Jv << "\n";
    //std::cout << "axis=\n" << axis << "\n";
    //std::cout << "T=\n" << T.matrix() << "\n";
    //std::cout << "J=\n" << J << "\n";
    //std::cout << "dq_ext=\n" << dq_ext << "\n";
    //std::cout << "dz=\n" << dz << "\n";

    // Return output
    ImpedanceCoordResult<3, 8> slider_coord;
    slider_coord.z = z;
    slider_coord.dz = dz;
    slider_coord.J = J;
    return slider_coord;
  }
};
