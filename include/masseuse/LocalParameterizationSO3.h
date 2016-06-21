
#ifndef LOCAL_PARAMETERIZATION_SO3_HPP
#define LOCAL_PARAMETERIZATION_SO3_HPP

#include <ceres/local_parameterization.h>
#include <sophus/so3.hpp>

namespace Sophus {
namespace compass {

class LocalParameterizationSO3 : public ceres::LocalParameterization {
public:
  virtual ~LocalParameterizationSO3() {}

  /**
   * \brief SO3 plus operation for Ceres
   *
   * \f$ R\cdot\exp(\widehat{\delta}) \f$
   */
  virtual bool Plus(const double * R_raw, const double * delta_raw,
                    double * R_plus_delta_raw) const {
    const Eigen::Map<const Sophus::SO3d> R(R_raw);
    const Eigen::Map<const Eigen::Matrix<double,3,1> > delta(delta_raw);
    Eigen::Map<Sophus::SO3d> R_plus_delta(R_plus_delta_raw);
    R_plus_delta = R * Sophus::SO3d::exp(delta);
    return true;
  }

  /**
   * \brief Jacobian of SO3 plus operation for Ceres
   *
   * \f$ \frac{\partial}{\partial \delta}R\cdot\exp(\widehat{\delta})|_{\delta=0} \f$
   */
  virtual bool ComputeJacobian(const double * R_raw, double * jacobian)
    const {
//    const Eigen::Map<const Sophus::SO3d> R(R_raw);
//    Eigen::Map<Eigen::Matrix<double,4,3,RowMajor> > jacobian(jacobian_raw);
//    //jacobian = R.internalJacobian().transpose();
//    Eigen::Matrix<double,4,3,RowMajor> jac_row_major = R.internalJacobian();
//    std::cerr << "calcualted Jacobian..." << std::endl;
//    jacobian = jac_row_major;

    /* Explicit formulation. Needs to be optimized */
    const double q1	   = R_raw[0];
    const double q2	   = R_raw[1];
    const double q3	   = R_raw[2];
    const double q0	   = R_raw[3];
    const double half_q0 = 0.5*q0;
    const double half_q1 = 0.5*q1;
    const double half_q2 = 0.5*q2;
    const double half_q3 = 0.5*q3;

    // d output_quaternion / d update
    jacobian[0] = half_q0;
    jacobian[1] = -half_q3;
    jacobian[2] = half_q2;

    jacobian[3] = half_q3;
    jacobian[4] = half_q0;
    jacobian[5] = -half_q1;

    jacobian[6] = -half_q2;
    jacobian[7] = half_q1;
    jacobian[8] = half_q0;

    jacobian[9] = -half_q1;
    jacobian[10] = -half_q2;
    jacobian[11] = -half_q3;
    return true;
  }

  virtual int GlobalSize() const {
    return Sophus::SO3d::num_parameters;
  }

  virtual int LocalSize() const {
    return Sophus::SO3d::DoF;
  }
};

}
}


#endif
