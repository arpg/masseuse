#include <masseuse/masseuse.h>
#include <iomanip>

namespace masseuse {

////////////////////////////////////////////////////////////////////////
const Values& Masseuse::GetValues(){
  return *values;
}

//////////////////////////////////////////////////////////////////////////
const Graph& Masseuse::GetGraph(){
  return *graph;
}

//////////////////////////////////////////////////////////////////////////
const std::vector<AbsPose>& Masseuse::GetComparisonPoses(){
  return comparison_pose_graph;
}

////////////////////////////////////////////////////////////////////////
const std::vector<AbsPose>& Masseuse::GetGroundTruth(){
  return gt_poses;
}

////////////////////////////////////////////////////////////////////////
void Masseuse::SaveAbsPG(string out_file) {
  if (output_abs_poses.size() == 0) {
    std::cerr << "No poses in the pose graph yet, skip\n";
    return;
  }

  std::cerr << "Saving " << output_abs_poses.size() << " absolute poses."
            << std::endl;

  // get the pose files name
  std::string sPoseFile = out_file;

  FILE *fp = fopen(sPoseFile.c_str(), "wb");

  // save the num of abs poses
  unsigned nAbsPos = relative_poses.size();
  fwrite(&nAbsPos, sizeof(unsigned), 1, fp);

  // save all abs poses
  for (unsigned i = 0; i != output_abs_poses.size(); i++) {
    AbsPose &aPos = output_abs_poses[i];
    fwrite(&aPos.id, sizeof(unsigned), 1, fp);
    fwrite(&aPos.pose_vec, sizeof(Eigen::Vector6d), 1, fp);
    fwrite(&aPos.cov, sizeof(Eigen::Matrix6d), 1, fp);
  }

  fclose(fp);
  std::cerr << "Finished Saving Pose Graph to " << sPoseFile
            << std::endl;
}


////////////////////////////////////////////////////////////////////////
void Masseuse::LoadGroundTruth(const string& gt_file){

  std::ifstream input_file(gt_file.c_str(), std::ifstream::in);


  if (!input_file.is_open()) {
    fprintf(stderr, "[LoadGroundTruth] Could not open file: %s\n",
            gt_file.c_str());
  }else{
    fprintf(stderr, "[LoadGroundTruth] Loading poses from file: %s\n",
            gt_file.c_str());
  }

  std::string line;
  while(std::getline(input_file, line)){
    AbsPose absPose;
    std::istringstream iss(line);
    if(iss >> absPose.pose_vec[0] >> absPose.pose_vec[1] >> absPose.pose_vec[2] >>
       absPose.pose_vec[3] >> absPose.pose_vec[4] >> absPose.pose_vec[5]){

      // Create an SE3 pose
      Point3 p(absPose.pose_vec.head<3>());
      Rot3 r(absPose.pose_vec[3], absPose.pose_vec[4], absPose.pose_vec[5]);
      Pose3 pose(r, p);
      absPose.Twp = pose;

      gt_poses.push_back(absPose);
    }
  }

  // Finished loading poses
  std::cerr << "[LoadGroundTruth] Read in "
            << gt_poses.size() << " ground truth poses" <<
               std::endl;

}

////////////////////////////////////////////////////////////////////////
void Masseuse::LoadPoseGraph(const string& pg_file){

  FILE *fp = (FILE *)fopen(pg_file.c_str(), "rb");


  if (fp == NULL) {
    fprintf(stderr, "Could not open file %s\n",
            pg_file.c_str());
  }

  unsigned numPoses = 0;
  if (fread(&numPoses, sizeof(unsigned), 1, fp) != 1) {
    printf("error! Cannot load num poses.\n");
    throw invalid_argument("LoadPg:  error loading file");
  }

  // save all abs poses
  for (unsigned i = 0; i != numPoses; i++) {
    AbsPose absPose;
    if (fread(&absPose.id, sizeof(unsigned), 1, fp) != 1) {
      throw invalid_argument("LoadPg:  error loading file");
    }
    if (fread(&absPose.pose_vec, sizeof(Eigen::Vector6d), 1, fp) != 1) {
      throw invalid_argument("LoadPg:  error loading file");
    }
    if (fread(&absPose.cov, sizeof(Eigen::Matrix6d), 1, fp) != 1) {
      throw invalid_argument("LoadPg:  error loading file");
    }

    // Create an SE3 pose
    Point3 p(absPose.pose_vec.head<3>());
    Rot3 r(absPose.pose_vec[3], absPose.pose_vec[4], absPose.pose_vec[5]);
    Pose3 Twp(r, p);
    absPose.Twp = Twp;

    comparison_pose_graph.push_back(absPose);
  }

}


////////////////////////////////////////////////////////////////////////
GraphAndValues Masseuse::LoadPoseGraphAndLCC(
    const string& pose_graph_file, bool read_lcc) {


  FILE *fp = (FILE *)fopen(pose_graph_file.c_str(), "rb");

  if (fp == NULL) {
    fprintf(stderr, "Could not open file %s\n",
            pose_graph_file.c_str());
  }


  if (fread(&origin, sizeof(Eigen::Vector6d), 1, fp) != 1) {
    throw invalid_argument("LoadPoseGraphAndLCC:  Cannot load the origin!");
  }

//  std::cerr << "read origin: " << origin.transpose() << std::endl;

  unsigned numRelPoses = 0;
  unsigned numLCC = 0;

  if (fread(&numRelPoses, sizeof(unsigned), 1, fp) != 1) {
    printf("error! Cannot load num of relative poses.\n");
    throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
  }

  if(read_lcc){
    if (fread(&numLCC, sizeof(unsigned), 1, fp) != 1) {
      printf("error! Cannot load num of loop closure constraints.\n");
      throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
    }
  }

  std::cerr << "Will load " << numRelPoses << " rel poses, "
            << numLCC << " loop closure constranits." << std::endl;

  relative_poses.clear();
  // save all rel poses
  for (unsigned i = 0; i != numRelPoses; i++) {
    RelPose rPos;
    if (fread(&rPos.ref_id, sizeof(unsigned), 1, fp) != 1) {
      throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
    }
    if (fread(&rPos.live_id, sizeof(unsigned), 1, fp) != 1) {
      throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
    }
    if (fread(&rPos.rel_pose, sizeof(Eigen::Vector6d), 1, fp) != 1) {
      throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
    }
    if (fread(&rPos.cov, sizeof(Eigen::Matrix6d), 1, fp) != 1) {
      throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
    }

    relative_poses.push_back(rPos);
  }

  if(read_lcc){
    loop_closure_constraints.clear();
    // save all lcc here
    for (unsigned i = 0; i != numLCC; i++) {
      RelPose rPos;
      if (fread(&rPos.ref_id, sizeof(unsigned), 1, fp) != 1) {
        throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
      }
      if (fread(&rPos.live_id, sizeof(unsigned), 1, fp) != 1) {
        throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
      }
      if (fread(&rPos.rel_pose, sizeof(Eigen::Vector6d), 1, fp) != 1) {
        throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
      }
      if (fread(&rPos.cov, sizeof(Eigen::Matrix6d), 1, fp) != 1) {
        throw invalid_argument("LoadPoseGraphAndLCC:  error loading file");
      }

      loop_closure_constraints.push_back(rPos);
    }
  }
  fclose(fp);

  std::cerr << "Finished loading Pose Graph from  " << pose_graph_file
            << std::endl;

  std::shared_ptr<Values> initial(new Values);
  std::shared_ptr<Graph> graph(new Graph);
  Pose3 prev_pose;

  unsigned numICPfailed = 0;
  for (size_t ii = 0; ii < relative_poses.size(); ++ii){
    RelPose curr_pose = relative_poses[ii];
    if(ii == 0){
      // first relative pose, initialize graph at origin
      unsigned id = curr_pose.ref_id;
      Rot3 R(origin[3], origin[4], origin[5]);
      Point3 t = origin.head<3>();
      Pose3 orig(R, t);
      (*initial)[id] = orig;
      prev_pose = orig;

//            std::cerr << "inserting origin: Rot: " << orig.rotationMatrix().eulerAngles
//                         (0,1,2).transpose() << " Trans: " << orig.translation().transpose() <<
//                         " at index:  " << id <<
//                         std::endl;
    }

    // Build the next vertex using the relative contstraint
    Rot3 R(curr_pose.rel_pose[3], curr_pose.rel_pose[4],
        curr_pose.rel_pose[5]);
    Point3 t = curr_pose.rel_pose.head<3>();
    Pose3 rel(R, t);

    Pose3 new_pose = prev_pose*rel;
    (*initial)[curr_pose.live_id] = new_pose;

    //    std::cerr << "inserting pose: Rot: " << new_pose.rotationMatrix().eulerAngles
    //                 (0,1,2).transpose() << " Trans: " << new_pose.translation().transpose() <<
    //                 " at index:  " << curr_pose.live_id <<
    //                 std::endl;

    prev_pose = new_pose;

    // Also insert a factor for the binary pose constraint
    unsigned id1 = curr_pose.ref_id;
    unsigned id2 = curr_pose.live_id;

    Matrix m = curr_pose.cov;
    if(m.sum() == 0 || m.determinant() <= 0 || std::isnan(m.determinant())
       || m.determinant() > options.cov_det_thresh){
      //      std::cerr << "ICP failed for rel pose between " << id1 << " and " << id2 <<
      //                   "Setting fixed covaraince..." <<
      //                   std::endl;
      Eigen::Vector6d cov_vec;
      // TODO: Improve handling of cases where the frame-to-frame ICP failed
      cov_vec << 7e-8, 7e-8, 7e-8, 8e-11, 8e-11, 8e-11;
      m = cov_vec.asDiagonal();
      numICPfailed++;
    }

    //    std::cerr <<  "Adding binary constraint between id: " << id1 << " and " <<
    //                  id2 << std::endl << "with cov det:" << m.determinant() << std::endl;

    // Create a new factor between poses
    if(options.use_identity_covariance){
      m.setIdentity();
    }

    m = m * options.rel_covariance_mult;

    Factor factor(id1, id2, rel, m);
    graph->push_back(factor);

  }
  std::cerr << "ICP failed " << numICPfailed << " times." << std::endl;

  if( read_lcc ){
    std::cerr << "Reading LLC." << std::endl;

    int discarded_lcc = 0;
    for(size_t ii = 0; ii < loop_closure_constraints.size(); ++ii){
      RelPose curr_lcc = loop_closure_constraints[ii];
      unsigned id1 = curr_lcc.ref_id;
      unsigned id2 = curr_lcc.live_id;

      Rot3 R(curr_lcc.rel_pose[3], curr_lcc.rel_pose[4],
          curr_lcc.rel_pose[5]);
      Point3 t = curr_lcc.rel_pose.head<3>();
      Pose3 lcc(R, t);

      Matrix m = curr_lcc.cov;

      if(m.sum() == 0 ||
         m.determinant() > options.cov_det_thresh ||
         m.determinant() <= 0 ||
         std::isnan(m.determinant()))
      {
        // ICP failed or something went wrong for this loop closure, ignoring
        // The determinant of the cov. matrix may also be larger than the
        // specified threshold.
        discarded_lcc++;
        continue;
      }

      //ZZZZZZZZZ Temp for specific dataset, remove this:
      //      std::set<int> remove_ids;
      //      int ids[] = {5880, 5850, 5840, 5830, 4020, 4000, 3990, 3980, 3950,
      //                   5800, 5810, 5830};
      //      remove_ids.insert(ids, ids+12);
      //      if(remove_ids.find(id1) != remove_ids.end() ||
      //         remove_ids.find(id2) != remove_ids.end()){
      //        std::cerr << "LLC with refID: " << id1 << " and liveID: " <<
      //                     id2 << " cov det: " << m.determinant() << std::endl;
      //        discarded_lcc++;
      //        continue;
      //      }


      // check if the lcc is between far away poses. If so, downweight it's
      // covariance.

      //      const Pose3* lcc_0 = dynamic_cast<const Pose3*>(&initial->at(id1));
      //      const Pose3* lcc_1 = dynamic_cast<const Pose3*>(&initial->at(id2));

      //      Pose3 lcc_diff = lcc_0->compose(lcc).inverse().compose(*lcc_1);
      //            std::cerr << "distance between poses for lcc: " << ii <<
      //                         " between poses " <<  id1 << " and " << id2 << ": "
      //                      << lcc_diff.translation().norm() << std::endl;
      //      Pose3 diff = (*lcc_0).inverse().compose(*lcc_1);
      //      std::cerr << "distance between poses for lcc: " << ii <<
      //                   " between poses " <<  id1 << " and " << id2 << ": "
      //                << lcc.translation().norm() << std::endl;



      //      SharedNoiseModel model = noiseModel::Gaussian::Information(m);
      //      NonlinearFactor::shared_ptr factor(
      //          new BetweenFactor<Pose3>(id1, id2, lcc, model));

      // Create a new factor between poses
      if(options.use_identity_covariance){
        m.setIdentity();
      }

      Factor lcc_factor(id1, id2, lcc, m);
      lcc_factor.isLCC = true;


      graph->push_back(lcc_factor);
      curr_lcc.ext_id = graph->size()-1;

      //m_addedLCC[keyFromId(id2)] = curr_lcc;

      /*

      // If we already have a constraint between poses i and j, only use the one
      // with the highest degree of certainty (lowest cov. determinant)
      if(m_addedLCC.count(keyFromId(id2)) == 0){
        // LCC has not been added yet, just add
        graph->push_back(factor);
        curr_lcc.m_nExtId = graph->size()-1;
        m_addedLCC[keyFromId(id2)] = curr_lcc;
      }else{
        // Check if the covariance of the new lcc for poses i and j
        // is smaller than what we are currently using
        EstPose old_lcc = m_addedLCC[keyFromId(id2)];
        if(old_lcc.m_Cov.determinant() > curr_lcc.m_Cov.determinant()){
          // new det is smaller, replace
          curr_lcc.m_nExtId = old_lcc.m_nExtId;
          graph->replace(old_lcc.m_nExtId, factor);
          m_addedLCC[keyFromId(id2)] = curr_lcc;
//          std::cerr << "determinant for new lcc between " << id1 << " and " <<
//                       id2 << " is: " << curr_lcc.m_Cov.determinant() << " < "
//                    << old_lcc.m_Cov.determinant() << std::endl;

        }else{
          // Determinant for new constraint is larger, skip it
          continue;
        }

      }
      */

    }

//    //ZZZZZZZZZZZZZ Temp, add wrong LCC to test switchable constraints
//    unsigned id1 = 480;
//    unsigned id2 = 870;
//    Pose3& T2 = initial->at(id2);

//    Pose3& T1 = initial->at(id1);

//    Pose3 T12 = T1.inverse() * T2;


//    // Add random values to the translation
//    T12.translation()[0] += 5;
//    T12.translation()[1] -= 2;
//    T12.translation()[3] += 7;

//    // Rotate the pose by an arbitrary amount
//    Sophus::SO3d rot(0.5, 0.7, 0.1);

//    T12.rotationMatrix() *= rot.matrix();
//    Factor wrong_lcc(id1, id2, T12, Eigen::Matrix6d::Identity());
//    wrong_lcc.isLCC = true;
//    // now add the wrong LCC to the graph
//    graph->push_back(wrong_lcc);


    std::cerr << "Did not use " << discarded_lcc << " LCC." << std::endl;

  }

  return make_pair(graph, initial);
}

////////////////////////////////////////////////////////////////////////
void Masseuse::LoadPosesFromFile(const string filename){
  std::tie(graph, values) = LoadPoseGraphAndLCC(filename, true);

  std::cerr << "Read in " << graph->size() << " factors and " <<
               values->size() << " poses." << std::endl;
}

////////////////////////////////////////////////////////////////////////
string Masseuse::KeyFromId(unsigned id){
  return std::to_string(id); /*+ "_" + std::to_string(id2);*/
}


////////////////////////////////////////////////////////////////////////
Masseuse::Masseuse(Options options) {
  this->options = options;
}


////////////////////////////////////////////////////////////////////////
void Masseuse::SaveResultsG2o(){
  //  std::string outputFile = options.g2o_output_dir +
  //      "/pose_graph_output.g2o";
  //  std::cout << "Writing results to file: " << outputFile
  //            << std::endl;
  //  writeG2o(*graph, result, outputFile);
  //  std::cout << "done! " << std::endl;

  //  if(options.save_ground_truth_g2o && gt_poses.size() > 0){

  //    NonlinearFactorGraph::shared_ptr gtGraph;
  //    Values gtPoses;
  //    std::size_t id = 0;
  //    for(AbsPose gtPose : gt_poses ){
  //      Rot3 R = Rot3::ypr(gtPose.pose[5], gtPose.pose[4], gtPose.pose[3]);
  //      Point3 t = Point3(gtPose.pose.head<3>());
  //      Pose3 pose(R, t);
  //      gtPoses.insert((Key)id++, pose);
  //    }

  //    //std::size_t found = outputFile.find_last_of("/");
  //    //std::string gtOutputFile = outputFile.substr(0, found+1) + "ground_truth.g2o";
  //    std::string gtOutputFile = options.g2o_output_dir + "/ground_truth.g2o";
  //    std::cout << "Writing ground truth to file: " << gtOutputFile << std::endl;
  //    writeG2o(*gtGraph, gtPoses, gtOutputFile);
  //    std::cout << "done! " << std::endl;
  //  }
}

////////////////////////////////////////////////////////////////////////
void Masseuse::PrintErrorStatistics(){
  // calclate the error
  Error err;
  if(!CalculateError(err)){
     std::cerr << "Unable to calculate error metrics." << std::endl;
     return;
  }

  std::cerr << "======================ERROR REPORT=====================" <<
               std::endl;
  std::cerr << "Average trans error (m): " << err.GetAverageTransError() <<
               std::endl;

  std::cerr << "Average rot error (deg): " << err.GetAverageRotError() <<
               std::endl;

  std::cerr << "Total distance traveled (m): " << err.DistanceTraveled() <<
               std::endl;

  std::cerr << "% Avg. trans error: " << err.GetPercentAverageTansError()
               * 100 << " %" << std::endl;

  std::cerr << "Max trans error (m): " << err.MaxTransError() << std::endl;

  std::cerr << "Max rot error (deg): " << err.MaxRotError() << std::endl;

  if(options.enable_switchable_constraints){
    int num_disabled_constraints = 0;
    int num_lcc = 0;
    for(const Factor& f : *graph){
      if(f.isLCC){
        num_lcc++;
        if(f.switch_variable < 0.1){
          num_disabled_constraints++;
        }
      }
    }
    fprintf(stderr, "Number of LCC that were disabled: %d ( %f%% )\n",
                      num_disabled_constraints,
            (float)num_disabled_constraints/(float)num_lcc * 100.0f
            );

  }

  std::cerr << "======================================================" <<
               std::endl;

}

////////////////////////////////////////////////////////////////////////
bool Masseuse::CalculateError(Error& error){

  // First check if we have a ground truth to compare against
  if(!gt_poses.size()){
    std::cerr << "Unable to calculate error, no ground truth provided." <<
                 std::endl;
    return false;
  }

  unsigned num_poses_to_compare = values->size();
  if(gt_poses.size() != values->size()){

    num_poses_to_compare = std::min(gt_poses.size(), values->size());
    std::cerr << "There are " << gt_poses.size() << " ground truth poses"
              << " and " << values->size() << " optimized poses. Will only" <<
                 " compare the first " << num_poses_to_compare << " poses." <<
                 std::endl;
  }

  if(num_poses_to_compare > 0){
    size_t index = 0;
    for(const auto& kvp : *values){

      if(index >= num_poses_to_compare){
        break;
      }

      Pose3 est_pose = kvp.second;
      Pose3 gt_pose = gt_poses.at(index).Twp;

      Eigen::Vector6d pose_error = (est_pose.inverse() * gt_pose).log();
      Eigen::Vector3d trans_error = pose_error.head<3>().cwiseAbs();
      Eigen::Vector3d rot_error = pose_error.tail<3>().cwiseAbs();
      error.Translation()+= trans_error;
      error.Rotation()+= rot_error;

      error.NumPoses()++;

      // Set the max trans and rotation errors
      if(error.MaxTransError() < trans_error.norm()){
        error.MaxTransError() = trans_error.norm();
      }
      if(error.MaxRotError() < rot_error.norm()){
        error.MaxRotError() = rot_error.norm();
      }

      if(index > 0){
        // add up the total distance traveled, based on the ground truth
        error.DistanceTraveled()+= (gt_poses.at(index-1).Twp.inverse() *
                                    gt_poses.at(index).Twp).translation().norm();
      }

      // calculate the % average translation error up to this point
      if(error.DistanceTraveled() > 0){
        error.PercentAvgTranslationError()+= (trans_error.norm() /
            error.DistanceTraveled());
      }

      index++;
    }
  }else{
    std::cerr << "No poses to compare." << std::endl;
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////
void Masseuse::Relax() {


  if(!graph->size() || !values->size()){
    std::cerr << "Pose graph not loaded. Load poses using 'LoadPosesFromFile'" <<
                 " before calling Relax()" << std::endl;
    throw runtime_error("Initial values or graph empty. Nothing to relax.");

  }

  // Global problem
  ceres::Problem problem;

  // Build the problem with the given pose graph

  if(options.enable_prior_at_origin){
    // Add a prior at the origin:
    Point3 p = origin.head<3>();
    Rot3 rot(origin[3], origin[4], origin[5]);
    Pose3 orig(rot, p);

    Eigen::Vector6d cov_vec;
    cov_vec << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    Eigen::MatrixXd m = cov_vec.asDiagonal();

    ceres::CostFunction* prior_cost_function =
        new ceres::AutoDiffCostFunction<PriorPoseCostFunctor<double>,
        Sophus::SE3::DoF,
        Sophus::SE3::num_parameters>
        (new PriorPoseCostFunctor<double>(orig, m.inverse().sqrt()));

    problem.AddResidualBlock(prior_cost_function, NULL,
                             values->begin()->second.data());

    //    std::cerr << "Adding prior at: " << orig.rotationMatrix().eulerAngles
    //                 (0,1,2).transpose() << " Trans: " <<
    //                 orig.translation().transpose() << std::endl << " to pose " <<
    //                 values->begin()->first << " : " <<
    //                 values->begin()->second.rotationMatrix().eulerAngles
    //                 (0,1,2).transpose() << " Trans: " <<
    //                 values->begin()->second.translation().transpose() << std::endl;
  }else{
    std::cerr << "Not adding any prior at origin" << std::endl;
  }

  if(options.fix_first_pose){
    problem.SetParameterBlockConstant(values->begin()->second.data());
  }

  // Now add a binary constraint for all relative and loop closure constraints
  for(Factor& f : *graph){

    Pose3& T_a = values->at(f.id1);
    Pose3& T_b = values->at(f.id2);

    if(options.optimize_rotations){
      // Full optimizaion over SE3

      if(options.enable_switchable_constraints && f.isLCC){
        // Use switchable constraints to selectively disable bad LCC's
        // during the optimization, see:
        // 'Switchable Constraints for Robust Pose Graph SLAM'
        ceres::CostFunction* binary_cost_function =
            new ceres::AutoDiffCostFunction<SwitchableBinaryPoseCostFunctor
            <double>, Sophus::SE3::DoF,
            Sophus::SE3::num_parameters,
            Sophus::SE3::num_parameters,
            1>
            (new SwitchableBinaryPoseCostFunctor<double>(f.rel_pose,
                                                         f.cov.inverse().sqrt()));


        HuberLoss* loss = new HuberLoss(options.huber_loss_delta);

        problem.AddResidualBlock(binary_cost_function, loss,
                                 T_a.data(),
                                 T_b.data(),
                                 &f.switch_variable);

        // Constrain the switch variable to be between 0 and 1
        problem.SetParameterLowerBound(&f.switch_variable, 0, 0.0);
        problem.SetParameterUpperBound(&f.switch_variable, 0, 1.0);

        // Add a prior to anchor the switch variable at its initial value
        ceres::CostFunction* prior_cost_function =
            new ceres::AutoDiffCostFunction<PriorCostFunctor<double>,
            1, 1>(new PriorCostFunctor<double>(f.switch_variable,
                                               options.switch_variable_prior_cov,
                                               0));

        problem.AddResidualBlock(prior_cost_function, NULL,
                                 &f.switch_variable);

      }else{

        ceres::CostFunction* binary_cost_function =
            new ceres::AutoDiffCostFunction<BinaryPoseCostFunctor
            <double>, Sophus::SE3::DoF,
            Sophus::SE3::num_parameters,
            Sophus::SE3::num_parameters>
            (new BinaryPoseCostFunctor<double>(f.rel_pose, f.cov.inverse().sqrt()));

        HuberLoss* loss = new HuberLoss(options.huber_loss_delta);

        problem.AddResidualBlock(binary_cost_function, loss,
                                 T_a.data(),
                                 T_b.data());
      }
    }else{
      // don't optimize over rotations, just include the translation in the
      // optimization

      ceres::CostFunction* binary_trans_cost_function =
          new ceres::AutoDiffCostFunction<BinaryTranslationCostFunctor
          <double>, 3, 3, 3>
          (new BinaryTranslationCostFunctor<double>(f.rel_pose.translation(),
                                                    f.cov.inverse().sqrt()));

      HuberLoss* loss = new HuberLoss(options.huber_loss_delta);

      problem.AddResidualBlock(binary_trans_cost_function, loss,
                               T_a.translation().data(),
                               T_b.translation().data());
    }

    if(options.enable_z_prior){
      // Add a prior on z so that it anchors the height to the initial value
      // this assumes roughly planar motion to avoid the z drift.

      double initial_z = origin[2]; // first three elements are x, y, z

      // The last parameter is the index of the z in the SO3 data structure
      // It is [i j k w x y z]
      ceres::CostFunction* prior_cost_function =
          new ceres::AutoDiffCostFunction<PriorCostFunctor<double>,
          1, 7>(new PriorCostFunctor<double>(initial_z,
                                             options.cov_z_prior,
                                             6));

      problem.AddResidualBlock(prior_cost_function, NULL,
                               T_b.data());

    }

  }

  ceres::LocalParameterization* local_param =
      new ceres::AutoDiffLocalParameterization
      <Sophus::masseuse::AutoDiffLocalParamSE3, 7, 6>;

  for(auto &pair : *values){
    if(problem.HasParameterBlock(pair.second.data())){
      problem.SetParameterization(pair.second.data(),
                                  local_param);
    }
  }


  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  ceres_options.minimizer_progress_to_stdout = options.print_minimizer_progress;
  ceres_options.max_num_iterations = options.num_iterations;
  ceres_options.update_state_every_iteration =
      options.update_state_every_iteration;
  ceres_options.check_gradients = options.check_gradients;
  ceres_options.function_tolerance = options.function_tolearnce;
  ceres_options.parameter_tolerance = options.parameter_tolerance;
  ceres_options.gradient_tolerance = options.gradient_tolerance;

  if(options.print_error_statistics){
    std::cerr << "BEFORE RELAXATION:" << std::endl;
    PrintErrorStatistics();
    std::cerr << std::endl;

//    if(options.enable_switchable_constraints){
//      std::cerr << "switch variables BEFORE optimizing: " << std::endl;
//      for(const Factor& f : *graph){
//        if(f.isLCC){
//          fprintf(stderr, "%1.3f ",
//                  f.switch_variable);
//        }
//      }
//      std::cerr << std::endl;
//    }

//    double initial_cost = 0.0;
//    std::vector<double> residuals(problem.NumResiduals());
//    problem.Evaluate(Problem::EvaluateOptions(), &initial_cost, &residuals
//                     , NULL, NULL);


//    std::cerr << "num residual blocks: " << problem.NumResidualBlocks() <<
//                 std::endl;
//    std::cerr << "Cost BEFORE optimizing: " << initial_cost << std::endl;

  }


  ceres::Solver::Summary summary;

  std::cerr << "Relaxing graph...." << std::endl;
  double ceres_time = masseuse::Tic();
  ceres::Solve(ceres_options, &problem, &summary);
  ceres_time = masseuse::Toc(ceres_time);
  fprintf(stderr, "Optimization finished in %fs \n",
                  ceres_time);

  if(options.print_full_report){
    std::cerr << summary.FullReport() << std::endl;
  }

  if(options.print_brief_report){
    std::cerr << summary.BriefReport() << std::endl;
  }


  if(options.print_error_statistics){
    std::cerr << std::endl << "AFTER REALXATION:" << std::endl;
    PrintErrorStatistics();

//    if(options.enable_switchable_constraints){
//      std::cerr << "switch variables AFTER optimizing: " << std::endl;
//      for(const Factor& f : *graph){
//        if(f.isLCC){
//          fprintf(stderr, "%1.3f ",
//                  f.switch_variable);
//        }
//      }
//      std::cerr << std::endl;
//    }

//    double final_cost = 0.0;
//    std::vector<double> residuals(problem.NumResiduals());
//    problem.Evaluate(Problem::EvaluateOptions(), &final_cost, &residuals,
//                     NULL, NULL);

    //std::cerr << "Cost AFTER optimizing: " << final_cost << std::endl;
    //    Eigen::Map<Eigen::VectorXd> vec_residuals(residuals.data(), residuals.size());
    //    std::cerr << "Residuals: " << vec_residuals << std::endl;

    problem.NumResiduals();
  }

  // Save optimization result in a binary file
  if(options.save_results_binary){
    output_abs_poses.clear();

    Eigen::Vector6d absPoseVec;
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    vector<pair<const double*, const double*> > covariance_blocks;


    for(const auto& kvp : *values){
      const Pose3& pose = kvp.second;
      Point3 p = pose.translation();
      Rot3 R = pose.so3();
      absPoseVec.head<3>() = p;
      // unpack Euler angles: roll, pitch, yaw
      absPoseVec.tail<3>() = R.matrix().eulerAngles(0, 1, 2);

      // Now get the covariance for this pose
      covariance_blocks.clear();
      covariance_blocks.push_back(std::make_pair(pose.data(), pose.data()));

      CHECK(covariance.Compute(covariance_blocks, &problem));

      double cov[6*6];
      covariance.GetCovarianceBlockInTangentSpace(
            pose.data(),
            pose.data(),
            cov);

      // convert to an Eigen matrix
      Eigen::Map < Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > cov_mat(
            cov);

      AbsPose absPose(kvp.first, absPoseVec, cov_mat);
      output_abs_poses.push_back(absPose);
    }

    SaveAbsPG(options.binary_output_path);

  }
}

}// namespace masseuse

