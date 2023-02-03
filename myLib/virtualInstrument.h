#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "myLib.h"

// #include <franka/exception.h>
// #include <franka/robot.h>
// #include <franka/model.h>

namespace virtualInstrument {

    const Eigen::Vector3d a1(0.0, 0.0, 1.0);
    const Eigen::Vector3d a1_dot(0.0, 0.0, 0.0);
    const Eigen::Vector3d X(1.0, 0.0, 0.0);

    struct kinematicsResult{
        Eigen::Vector3d r; // origin of instrument frame
        Eigen::Matrix3d R; // Rotation matrix from instrument frame to world frame
        Eigen::Affine3d T; // Transform from instrument frame to world frame
        Eigen::Vector3d a2; // joint 2 axis
        Eigen::Vector3d a3; // joint 3 axis
        Eigen::Matrix<double, 6, 3> J; // geometric jacobian
        Eigen::Vector3d w; // Angular velocity
        Eigen::Vector3d v; // linear velocity
    };

    struct dynamicsResult{
        Eigen::Vector3d a2_dot;
        Eigen::Vector3d a3_dot;
        Eigen::Matrix3d R_dot;
        Eigen::Matrix<double, 6, 3> J_dot;
        Eigen::Matrix<double, 3, 3> M;
        Eigen::Vector3d vp;
    };

    class virtualInstrument{
      public:
        Eigen::Vector3d origin;
        double m; // total mass
        Eigen::Matrix3d I0; // inertia
        Eigen::Vector3d q;
        Eigen::Vector3d q_dot;

        kinematicsResult forwardKinematics() {
            kinematicsResult ret;
            double q1 = q[0], q2 = q[1], q3 = q[2];
            double d, x, y, z;
            d =  q3 * cos(q2);
            ret.r = Eigen::Vector3d(
                d  * cos(q1),
                d  * sin(q1),
            -q3 * sin(q2)
            );
            ret.a2 = Eigen::Vector3d(
                -sin(q1),
                cos(q1),
                0.
            );
            ret.a3 = Eigen::Vector3d(
                cos(q1) * cos(q2),
                sin(q1) * cos(q2),
                sin(q2)
            );

            ret.R.col(0) = ret.a3;
            ret.R.col(1) = ret.a2;
            ret.R.col(2) = ret.a3.cross(ret.a2);

            Eigen::Matrix4d T;
            T.setIdentity();
            T.block<3,3>(0,0) = ret.R;
            T.block<3,1>(0,3) = ret.r;
            ret.T = Eigen::Affine3d(T);

            ret.J.block<3, 1>(0, 0) = a1.cross(ret.r);
            ret.J.block<3, 1>(0, 1) = ret.a2.cross(ret.r);
            ret.J.block<3, 1>(0, 2) = ret.a3;
            ret.J.block<3, 1>(3, 0) = a1;
            ret.J.block<3, 1>(3, 1) = ret.a2.cross(ret.r);
            ret.J.block<3, 1>(3, 2) = Eigen::Vector3d(0., 0., 0.);
            
            Eigen::Matrix<double, 6, 1> twist(ret.J * q_dot);
            ret.w = twist.head(3);
            ret.v = twist.tail(3);
            return ret;
        }        
        
        dynamicsResult dynamics(kinematicsResult kr){
            dynamicsResult ret;
            double q1 = q[0], q2 = q[1], q3 = q[2];
            double q1_dot = q_dot[0], q2_dot = q_dot[1], q3_dot = q_dot[2];
            ret.a2_dot = Eigen::Vector3d(
                -cos(q1) * q1_dot,
                -sin(q1) * q1_dot,
                0.
            );
            ret.a3_dot = Eigen::Vector3d(
                -sin(q1)*cos(q2)*q1_dot - cos(q1)*sin(q2)*q2_dot,
                cos(q1)*cos(q2)*q1_dot - sin(q1)*sin(q2)*q2_dot,
                cos(q2)*q2_dot
            );

            ret.R_dot.col(0) = ret.a3_dot;
            ret.R_dot.col(1) = ret.a2_dot;
            // As R_dot is skew symmetric, we can infer...
            ret.R_dot.col(2) = Eigen::Vector3d(-ret.R_dot(2, 0), - ret.R_dot(2, 1), 0.);

            ret.J_dot.block<3, 1>(0, 0) = kr.v.cross(a1);
            ret.J_dot.block<3, 1>(0, 1) = kr.v.cross(kr.a2) + kr.r.cross(ret.a2_dot);
            ret.J_dot.block<3, 1>(0, 2) = kr.v.cross(kr.a3) + kr.r.cross(ret.a3_dot);
            ret.J_dot.block<3, 1>(3, 0) = a1_dot;
            ret.J_dot.block<3, 1>(3, 1) = ret.a2_dot;
            ret.J_dot.block<3, 1>(3, 2) = Eigen::Vector3d(0., 0., 0.);
            

            Eigen::Matrix<double, 3, 3> Jv = kr.J.topRows(3);
            Eigen::Matrix<double, 3, 3> Jw = kr.R * kr.J.bottomRows(3); // Angular velocity jacobian rotated to instrument frame
            
            Eigen::Matrix<double, 3, 3> Jv_dot = ret.J_dot.topRows(3);
            Eigen::Matrix<double, 3, 3> Jw_dot = kr.R * ret.J_dot.bottomRows(3) +  ret.R_dot * kr.J.bottomRows(3); // TODO check this is right
            
            ret.M = 0.5 * (Jv.transpose()*Jv*m + Jw.transpose()*I0*Jw);
            ret.vp = (Jv.transpose()*Jv_dot*m + Jw.transpose()*I0*Jw_dot) * q_dot;
        }

        void update(kinematicsResult kr, Eigen::Vector3d u, double dt) {
            dynamicsResult dr = dynamics(kr); // reuse kinematics result in computations
            Eigen::Vector3d q_ddot = dr.M.inverse() * (u + dr.vp);
            q += q_dot * dt;
            q_dot += q_ddot * dt;
        }
    };   
}