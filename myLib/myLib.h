#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>

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
  Eigen::Matrix<double, Dim, NDOF> jacobian;
};

struct ImpedanceCoordArgs {
  Eigen::Affine3d transform;
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq;  // joint velocities
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> J;   // gemoetric_jacobian
}

template <int Dim, int NDOF>
class DiagonalSpringDamper {
 public:
  Eigen::Array<double, Dim> stiffness;
  Eigen::Array<double, Dim> damping;
  Eigen::Vector<double, NDOF> F(ImpedanceCoordResult<Dim, NDOF> coord) {
      return coord.J.transpose() * (-stiffness * coord.z.array() - damping * coord.dz.array()))
  }
};

ImpedanceCoordResult<2, 7> computePortCoord(ImpedanceCoordArgs iargs, PortCoord port);
ImpedanceCoordResult<3, 7> computeWorldCoord(ImpedanceCoordArgs iargs, WorldCoord world);

Eigen::Vector3d register_point(franka::Robot robot, Eigen::Vector3d offset);

class VirtualPrismaticJoint {
 private:
  Franka::frame frame;
  Eigen::Affine3d A;
  double q;
  double qdot;
  double inertance;
  double damping;

 public:
  VirtualPrismaticJoint(#TODO) {}
  void update(double F, double dt) {
    double qdd = (F - damping * qdot) / inertance;
    q += qdot * dt;
    qdot += qdd * dt;
  }
  ImpedanceCoordResult<3, 8> computeCoord(ImpedanceCoordArgs iargs) {
    // Get prismatic joint axis in world frame
    Eigen::Affine3d T = iargs.transform * A;
    Eigen::Vector3d axis = T.linear() * Eigen::Vector3d({0.0, 0.0, 1.0});

    // Get slider position in world frame
    Eigen::Vector3d offset = q * axis;
    Eigen::Vector3d z << T.translation() + offset;

    // Get offset jacobian for slider position
    Eigen::Matrix<double, 3, 7> offset_Jv = offset_jacobian(iargs.transform, iargs.J, offset);

    // Add row
    Eigen::Matrix<double, 3, 7> J;
    J.leftCols(7) << offset_Jv;
    J.rightCols(1) << axis;

    // Return output
    ImpedanceCoordResult<3, 8> slider_coord;
    slider_coord.z = z;
    slider_coord.dz = J * iargs.dq;
    slider_coord.jacobian = J;
    return slider_coord;
  }
};
