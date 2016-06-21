#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <fstream>
#include <tuple>
#include <stdio.h>
#include <eigen3/Eigen/Core>
#include "CeresCostFunctions.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "AutoDiffLocalParamSE3.h"

using namespace std;

namespace Eigen{
    typedef Matrix<double, 6, 1> Vector6d;
    typedef Matrix<double, 6, 6> Matrix6d;
}

namespace masseuse {

class Factor;

// Convenience typedefs
typedef Sophus::SE3d Pose3;
typedef Sophus::SO3d Rot3;
typedef Eigen::Vector3d Point3;
typedef Eigen::MatrixXd Matrix;
typedef std::map<unsigned, Pose3> Values;
typedef std::vector<Factor> Graph;
typedef std::pair<std::shared_ptr<Graph>, std::shared_ptr<Values>>
GraphAndValues;

///////////////////////////////////////////////////////////////////////////////
class Factor {
  public:
  Factor(){

  }

  Factor(unsigned id1, unsigned id2, Pose3 rel, Matrix cov)
    : id1(id1), id2(id2), rel_pose(rel), cov(cov){
  }

  unsigned id1;
  unsigned id2;
  Pose3 rel_pose;
  Matrix cov;
  bool isLCC = false;
};

///////////////////////////////////////////////////////////////////////////////
class AbsPose {
 public:
  AbsPose(){
    id = -1;
    pose_vec.setZero();
    cov.setZero();
  }

  AbsPose(unsigned id, Eigen::Vector6d pose,
          Eigen::Matrix6d cov)
    : id(id), pose_vec(pose), cov(cov)
  {
  }

  unsigned id;
  Eigen::Vector6d pose_vec;
  Pose3 Twp;
  Eigen::Matrix6d cov;
};

///////////////////////////////////////////////////////////////////////////////
class RelPose {
 public:
  RelPose(){
    live_id = -1;
    ref_id = -1;
    rel_pose.setZero();
    cov.setZero();
    ext_id = -1;
  }

  RelPose(unsigned ref_id, unsigned live_id, Eigen::Vector6d rel_pose,
          Eigen::Matrix6d cov)
    : ref_id(ref_id), live_id(live_id), rel_pose(rel_pose), cov(cov)
  {
  }

  unsigned ext_id;
  unsigned ref_id;
  unsigned live_id;
  Eigen::Vector6d rel_pose;
  Eigen::Matrix6d cov;
};

///////////////////////////////////////////////////////////////////////////////
struct Options
{
  bool save_initial_values_g2o = false;
  bool print_full_report = true;
  bool print_minimizer_progress = false;
  bool save_results_binary = false;
  double abs_error_tol = 1e-15;
  double rel_error_tol = 1e-15;
  double rel_covariance_mult = 1;
  double cov_det_thresh = 1e-35;
  string binary_output_path = "";
  string g2o_output_dir = "";
  bool save_ground_truth_g2o = false;
  bool do_switchable_constraints = true;
  bool optimize_rotations = true;
  int num_iterations = 1000;
  bool enable_prior_at_origin = true;
};

///////////////////////////////////////////////////////////////////////////////
class Masseuse
{
public:
    Masseuse() {}
    Masseuse(Options options);
    void LoadPosesFromFile(const string filename);
    void SaveAbsPG(string out_file);
    void SaveResultsG2o();
    const Values& GetValues();
    const Graph& GetGraph();
    const std::vector<AbsPose>& GetGroundTruth();
    void Relax();
//    GraphAndValues LoadPoseGraph(const string& pose_graph_file,
//                                 bool save_to_g2o);
    void LoadGroundTruth(const string& gt_file);

    Options options;
private:
    GraphAndValues LoadPoseGraphAndLCC(const string& pose_graph_file,
                                       bool read_lcc);
    string KeyFromId(unsigned id);

    std::vector<RelPose> relative_poses;
    std::vector<AbsPose> output_abs_poses;
    std::map<string, RelPose> added_LCC;
    std::vector<RelPose> loop_closure_constraints;
    std::vector<AbsPose> gt_poses;
    Eigen::Vector6d origin;
    std::shared_ptr<Graph> graph;
    std::shared_ptr<Values> values;

}; //Masseuse

}
