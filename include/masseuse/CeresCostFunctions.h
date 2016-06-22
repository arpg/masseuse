#pragma once

#include <iostream>
#include <iomanip>
#include <fstream>
#include <unsupported/Eigen/MatrixFunctions>
#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include "Utils.h"

using ceres::SizedCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::HuberLoss;

namespace masseuse {

////////////////////////////////////////////////////////////////////////////////
struct TestCostFunctor
{
  TestCostFunctor(Sophus::SE3d T_aw)
    : T_aw(T_aw)
  {
  }

  template<typename T>
  bool operator()( const T* const sT_wa, T* sResiduals ) const
  {
    const Eigen::Map<const Sophus::SE3Group<T> > T_wa(sT_wa);
    Eigen::Map<Eigen::Matrix<T,6,1> > residuals(sResiduals);

    residuals = (T_aw.cast<T>() * T_wa).log();
    return true;
  }

  Sophus::SE3d T_aw;
};


////////////////////////////////////////////////////////////////////////////////
class TestSE3CostFunction :
    public ceres::SizedCostFunction<6,
    Sophus::SE3d::num_parameters>
{

private:
  Sophus::SE3d constraint;

public:
  virtual ~TestSE3CostFunction() {}

  TestSE3CostFunction(Sophus::SE3d& pose):
    constraint(pose){
  }

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {

    Eigen::Map < Eigen::Matrix<double, 6, 1> > residuals_(residuals);
    const Eigen::Map<const Sophus::SE3Group<double> > t_w1_(parameters[0]);
    const Sophus::SE3d t_w1(t_w1_);

    //    std::cerr << "pose_: \n" << t_w1_.matrix() << std::endl;

    //    for(int i = 0; i < 7; ++i){
    //      std::cerr << "pose_[" << i << "]: " << ((Sophus::SE3Group<double>)
    //                   t_w1_).data()[i] << std::endl;
    //    }

    residuals_ = log_decoupled(t_w1, constraint);


    if(jacobians != NULL && jacobians[0] != NULL){

      Eigen::Matrix<double, 6, 7> dz_dx;
      dz_dx.setZero();
      dz_dx = dLog_decoupled_dt1Ceres(t_w1, constraint);
      //      std::cerr << "dz_dx: \n" << dz_dx << std::endl;

      Eigen::Map < Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > j_x(
            jacobians[0]);
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> dz_dx_rowMajor;
      j_x = dz_dx;

      //      std::cerr << "j_x:\n" << std::endl;
      //      for(int i = 0; i < 6; ++i){// row
      //        for(int j = 0; j < 7; ++j){//colum
      //          std::cerr << jacobians[0][7*i + j] << " ";
      //        }
      //        std::cerr << std::endl;
      //      }

    }

    return true;
  }
};

////////////////////////////////////////////////////////////////////////////////
struct TestAutoDiffSE3CostFunctor {
  TestAutoDiffSE3CostFunctor(const Sophus::SE3d& pose):
    constraint(pose){
  }

  template<typename T>
  bool operator()(const T* const pose, T* residuals) const{
    const Eigen::Map< const Sophus::SE3Group<T> > pose_(pose);
    Eigen::Map< Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);

    pose_residuals = (pose_ * (constraint.cast<T>()).inverse()).log();
    return true;
  }

  Sophus::SE3d constraint;
};


////////////////////////////////////////////////////////////////////////////////
template<typename Scalar = double>
struct BinaryTranslationCostFunctor {
  BinaryTranslationCostFunctor(const Eigen::Matrix<Scalar, 3, 1>& trans,
                        const Eigen::Matrix<Scalar, 3, 3> cov = NULL)
    : trans_measured(trans),
      cov_inv_sqrt(cov)
  {
  }

  template<typename T>
  bool operator()(const T* const trans1, const T* const trans2,
                  T* residuals) const{

    // Pose pair to optimize over
    const Eigen::Map< const Eigen::Matrix<T, 3, 1> > trans_a(trans1);
    const Eigen::Map< const Eigen::Matrix<T, 3, 1> > trans_b(trans2);

    // Residual vector in tangent space
    Eigen::Map< Eigen::Matrix<T, 3, 1> > trans_residuals(residuals);

    trans_residuals = cov_inv_sqrt.template cast<T>() *
        (trans_measured.template cast<T>() - (trans_b - trans_a));

    return true;
  }

  Eigen::Matrix<Scalar, 3, 1> trans_measured;
  // Square root inverse of the covariance matrix
  const Eigen::Matrix<Scalar, 3, 3> cov_inv_sqrt;

};


////////////////////////////////////////////////////////////////////////////////
template<typename Scalar = double>
struct BinaryPoseCostFunctor {
  BinaryPoseCostFunctor(const Sophus::SE3d& rel_pose,
                        const Eigen::Matrix<Scalar, 6, 6> cov = NULL)
    : Tab_est(rel_pose),
      cov_inv_sqrt(cov)
  {
  }

  template<typename T>
  bool operator()(const T* const pose1, const T* const pose2,
                  T* residuals) const{

    // Pose pair to optimize over
    const Eigen::Map< const Sophus::SE3Group<T> > pose_a(pose1);
    const Eigen::Map< const Sophus::SE3Group<T> > pose_b(pose2);

    // Residual vector in tangent space
    Eigen::Map< Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);

    Sophus::SE3Group<T> Tab_meas = pose_a.inverse() * pose_b;
    pose_residuals = cov_inv_sqrt.template cast<T>() *
        log_decoupled(Tab_meas, Tab_est.cast<T>());

    return true;
  }

  Sophus::SE3d Tab_est;
  // Square root inverse of the covariance matrix, in tangent space
  const Eigen::Matrix<Scalar, 6, 6> cov_inv_sqrt;

};

////////////////////////////////////////////////////////////////////////////////
template<typename Scalar = double>
struct PriorCostFunctor {
  PriorCostFunctor(const double& p,
                   const double& xi = 1,
                   const int &val_index = 0)
    :p(p),
     xi(xi),
     val_index(val_index)
  {
  }

  template<typename T>
  bool operator()(const T* const s, T* residual) const{
      residual[0] = (T)xi * (s[val_index] - (T)p);
      return true;
  }

  double p;
  double xi;
  int val_index;
};

////////////////////////////////////////////////////////////////////////////////
template<typename Scalar = double>
struct SwitchableBinaryPoseCostFunctor {
  SwitchableBinaryPoseCostFunctor(const Sophus::SE3d& rel_pose,
                        const Eigen::Matrix<Scalar, 6, 6> cov = NULL)
    : Tab_est(rel_pose),
      cov_inv_sqrt(cov)
  {
  }

  template<typename T>
  bool operator()(const T* const pose1, const T* const pose2,
                  const T* const s, T* residuals) const{

    // Pose pair to optimize over
    const Eigen::Map< const Sophus::SE3Group<T> > pose_a(pose1);
    const Eigen::Map< const Sophus::SE3Group<T> > pose_b(pose2);

    // Residual vector in tangent space
    Eigen::Map< Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);

    Sophus::SE3Group<T> Tab_meas = pose_a.inverse() * pose_b;
    pose_residuals = s[0] * cov_inv_sqrt.template cast<T>() *
        log_decoupled(Tab_meas, Tab_est.cast<T>());

    return true;
  }

  Sophus::SE3d Tab_est;
  // Square root inverse of the covariance matrix, in tangent space
  const Eigen::Matrix<Scalar, 6, 6> cov_inv_sqrt;

};


////////////////////////////////////////////////////////////////////////////////
template<typename Scalar = double>
struct PriorPoseCostFunctor {
  PriorPoseCostFunctor(const Sophus::SE3d& pose,
                        const Eigen::Matrix<Scalar, 6, 6> cov = NULL)
    : T_wv(pose),
      cov_inv_sqrt(cov)
  {
  }

  template<typename T>
  bool operator()(const T* const pose, T* residuals) const{
    // Pose to optimize over
    const Eigen::Map< const Sophus::SE3Group<T> > pose_(pose);
    const Sophus::SE3Group<T> p(pose_);

    // Residual vector in tangent space
    Eigen::Map< Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);

    pose_residuals = cov_inv_sqrt.template cast<T>() *
        log_decoupled(p, T_wv.cast<T>());
    return true;
  }

  Sophus::SE3d T_wv;
  // Square root inverse of the covariance matrix, in tangent space
  const Eigen::Matrix<Scalar, 6, 6> cov_inv_sqrt;

};


} // namespace masseuse
