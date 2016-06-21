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

////////////////////////////////////////////////////////////////////////
const std::vector<AbsPose>& Masseuse::GetGroundTruth(){
  return gt_poses;
}

////////////////////////////////////////////////////////////////////////
void Masseuse::SaveAbsPG(string out_file) {
  if (output_abs_poses.size() == 0) {
    std::cout << "No poses in the pose graph yet, skip\n";
    return;
  }

  std::cout << "Saving " << output_abs_poses.size() << " absolute poses."
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
  std::cout << "Finished Saving Pose Graph to " << sPoseFile
            << std::endl;
}

////////////////////////////////////////////////////////////////////////
//GraphAndValues Masseuse::LoadPoseGraph(const string& pose_graph_file,
//                                           bool save_to_g2o) {

//  abs_poses.clear();

//  FILE *fp = (FILE *)fopen(pose_graph_file.c_str(), "rb");

//  if (fp == NULL) {
//    fprintf(stderr, "Could not open file %s\n",
//            pose_graph_file.c_str());
//  }

//  unsigned numAbsPoses = 0;

//  if (fread(&numAbsPoses, sizeof(unsigned), 1, fp) != 1) {
//    printf("err! cannot load number of absolute poses.\n");
//     throw invalid_argument("loadPG_3D: error loading file");
//  }

//  std::cout << "Will load " << numAbsPoses << " abs poses. "
//            << std::endl;

//  // save all abs poses
//  for (unsigned i = 0; i != numAbsPoses; i++) {
//    AbsPose absPos;
//    if (fread(&absPos.id, sizeof(unsigned), 1, fp) != 1) {
//       throw invalid_argument("loadPG_3D:  error loading file");
//    }
//    if (fread(&absPos.pose, sizeof(Eigen::Vector6d), 1, fp) != 1) {
//       throw invalid_argument("loadPG_3D:  error loading file");
//    }
//    if (fread(&absPos.cov, sizeof(Eigen::Matrix6d), 1, fp) != 1) {
//       throw invalid_argument("loadPG_3D:  error loading file");
//    }
//    abs_poses.push_back(absPos);
////    std::cout << "absPose: id: " << absPos.id << ", rel_pos: " <<
////                 absPos.m_AbsPos.transpose()
////              << ", cov:\n" << absPos.m_Cov << std::endl;

//    //
//  }

//  fclose(fp);
//  std::cout << "[LoadPG] Finish loading Pose Graph from  " << pose_graph_file
//            << std::endl << " read " << abs_poses.size() << " poses" <<
//               std::endl;

//  Values::shared_ptr initial(new Values);
//  NonlinearFactorGraph::shared_ptr graph(new NonlinearFactorGraph);

//  Pose3 prev_pose;
//  for (unsigned ii = 0; ii < abs_poses.size(); ++ii){
//    AbsPose curr_pose = abs_poses[ii];

//    // Build the next vertex using the relative contstraint

//    Rot3 R = Rot3::ypr(curr_pose.pose[5], curr_pose.pose[4],
//        curr_pose.pose[3]);
//    Point3 t = Point3(curr_pose.pose.head<3>());
//    Pose3 newPose(R, t);
//    initial->insert((Key)curr_pose.id, newPose);

//    // Also insert a factor for the binary pose constraint
//    if(ii > 0){
//      Key id1 = abs_poses[ii-1].id;
//      Key id2 = curr_pose.id;

//      Pose3 rel = prev_pose.inverse().compose(newPose);
//      Matrix m = curr_pose.cov;

//      SharedNoiseModel model = noiseModel::Gaussian::Information(m);
//      NonlinearFactor::shared_ptr factor(
//          new BetweenFactor<Pose3>(id1, id2, rel, model));
//      graph->push_back(factor);
//    }
//    prev_pose = newPose;
//  }

//  if(save_to_g2o){
//    writeG2o(*graph, *initial, "relaxed_poses.g2o");
//  }

//  return make_pair(graph, initial);
//}

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
  std::cout << "[LoadGroundTruth] Read in "
            << gt_poses.size() << " ground truth poses" <<
               std::endl;

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

  std::cerr << "read origin: " << origin.transpose() << std::endl;

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

  std::cout << "Will load " << numRelPoses << " rel poses, "
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

  std::cout << "Finished loading Pose Graph from  " << pose_graph_file
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

//      std::cerr << "inserting origin: Rot: " << orig.rotationMatrix().eulerAngles
//                   (0,1,2).transpose() << " Trans: " << orig.translation().transpose() <<
//                   " at index:  " << id <<
//                   std::endl;
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


    m = m * options.rel_covariance_mult;
//    std::cerr <<  "Adding binary constraint between id: " << id1 << " and " <<
//                  id2 << std::endl << "with cov det:" << m.determinant() << std::endl;

    // Create a new factor between poses
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
      Factor lcc_factor(id1, id2, lcc, m);
      lcc_factor.isLCC = true;


      graph->push_back(lcc_factor);
      curr_lcc.ext_id = graph->size()-1;

      if(options.do_switchable_constraints){
        // Create a new switch variable

      }
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
    std::cerr << "Did not use " << discarded_lcc << "LCC." << std::endl;

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
void Masseuse::Relax() {


  if(!graph->size() || !values->size()){
    std::cerr << "Pose graph no loaded. Load poses using 'LoadPosesFromFile'" <<
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

    std::cerr << "Adding prior at: " << orig.rotationMatrix().eulerAngles
                                  (0,1,2).transpose() << " Trans: " <<
                 orig.translation().transpose() << std::endl << " to pose " <<
                 values->begin()->first << " : " <<
                 values->begin()->second.rotationMatrix().eulerAngles
                 (0,1,2).transpose() << " Trans: " <<
  values->begin()->second.translation().transpose() << std::endl;
  }else{
    std::cerr << "Not adding any prior at origin" << std::endl;
  }


  // Now add a binary constraint for all relative and loop closure constraints
  for(Factor f : *graph){
      Pose3& T_a = values->at(f.id1);
      Pose3& T_b = values->at(f.id2);

    ceres::CostFunction* binary_cost_function =
        new ceres::AutoDiffCostFunction<BinaryPoseCostFunctor
        <double>, Sophus::SE3::DoF,
        Sophus::SE3::num_parameters,
        Sophus::SE3::num_parameters>
        (new BinaryPoseCostFunctor<double>(f.rel_pose, f.cov.inverse().sqrt()));

    problem.AddResidualBlock(binary_cost_function, NULL,
                             T_a.data(),
                             T_b.data());

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



  std::cout << "Optimizing the factor graph" << std::endl;
  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres_options.minimizer_progress_to_stdout = options.print_minimizer_progress;
  ceres_options.max_num_iterations = options.num_iterations;
  ceres_options.update_state_every_iteration = false;
  ceres_options.check_gradients = false;

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_options, &problem, &summary);

  if(options.print_full_report){
    std::cerr << summary.FullReport() << std::endl;
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

