#include <iostream>

#include <masseuse/masseuse.h>
#include <CVars/CVar.h>
#include "masseuse_gui-cvars.h"
#include <glog/logging.h>

/*-----GUI Includes-----------*/
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include "../libGUI/AnalyticsView.h"
#include "../libGUI/GLPathRel.h"
#include "../libGUI/GLPathAbs.h"
#include "../libGUI/GLPathSegment.h"
/*----------------------------*/


/*-----------COMMAND LINE FLAGS-----------------------------------------------*/
DEFINE_string(out, "", "path to save output file");
DEFINE_string(in, "", "input poses file (relative poses)");
DEFINE_string(gt, "", "ground truth poses (absolute)");
DEFINE_string(g2o, "", "output directory for g2o files");

/*----------------------------------------------------------------------------*/

using namespace std;

struct GuiVars{
  SceneGraph::GLSceneGraph gl_graph;
  pangolin::View view_3d;
  GLPathAbs gl_path_gt;
  GLPathAbs gl_path_input;
  GLPathAbs gl_path_output;
  GLPathSegment gl_segment_lcc;
  SceneGraph::GLGrid gl_grid;
};

GuiVars gui_vars;

void LoadPosesFromFile(std::shared_ptr<masseuse::Masseuse> relaxer);

///////////////////////////////////////////////////////////////////////////////
void InitGui(const std::shared_ptr<masseuse::Masseuse>
             pgr){
  pangolin::Var<bool>::Attach("masseuse.OptimizeRotations",
                              pgr->options.optimize_rotations, true, true);
  pangolin::Var<bool>::Attach("masseuse.EnableSwitchableConstraints",
                              pgr->options.do_switchable_constraints, false, true);
  pangolin::Var<bool>::Attach("masseuse.PrintMinimizerProgress",
                              pgr->options.print_minimizer_progress, false, true);
  pangolin::Var<bool>::Attach("masseuse.PrintFullReport",
                              pgr->options.print_full_report, true, true);
  pangolin::Var<bool>::Attach("masseuse.EnablePriorAtOrigin",
                              pgr->options.enable_prior_at_origin, true, true);
  pangolin::Var<bool>::Attach("masseuse.SaveOutputBinary",
                              pgr->options.save_results_binary, true, true);
  pangolin::Var<double>::Attach("masseuse.StiffnessFactor",
                              pgr->options.rel_covariance_mult);
  pangolin::Var<double>::Attach("masseuse.CovarianceDeterminantThreshold",
                              pgr->options.cov_det_thresh);
  pangolin::Var<int>::Attach("masseuse.NumIterations",
                              pgr->options.num_iterations);



}

///////////////////////////////////////////////////////////////////////////////
void Run(const std::shared_ptr<masseuse::Masseuse>
         pgr){

  pangolin::CreateWindowAndBind("Masseuse", 1600, 800);

  // Set up panel.
  const unsigned int panel_size = 180;

  pangolin::CreatePanel("ui").SetBounds(0, 1, 0,
                                        pangolin::Attach::Pix(panel_size));

  pangolin::Var<bool>           ui_show_gt_path("ui.Show GT Path", false, true);
  pangolin::Var<bool>           ui_show_initial_path("ui.Show Initial Path", true, true);
  pangolin::Var<bool>           ui_show_relaxed_path("ui.Show Relaxed Path", true, true);
  pangolin::Var<bool>           ui_show_lcc_segments("ui.Show Loop Closures", false, true);
  pangolin::Var<bool>           ui_show_lcc_poses("ui.Show Loop Closure Poses", false, true);
  pangolin::Var<bool>           ui_relax("ui.Relax", true, false);

  // Set up container.
  pangolin::View& container = pangolin::CreateDisplay();
  container.SetBounds(0, 1, pangolin::Attach::Pix(panel_size), 0.75);
  container.SetLayout(pangolin::LayoutEqual);
  pangolin::DisplayBase().AddDisplay(container);

  // Set up 3D view for container.
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();

  // Set background color to black.
  glClearColor(0, 0, 0, 1);

  // Configure Paths
  gui_vars.gl_path_gt.SetPoseDisplay(0);
  gui_vars.gl_path_gt.SetLineColor(0.0, 1.0, 0.0);

  gui_vars.gl_path_input.SetPoseDisplay(0);
  gui_vars.gl_path_input.SetLineColor(0.0, 0.0, 1.0);

  gui_vars.gl_path_output.SetPoseDisplay(0);
  gui_vars.gl_path_output.SetLineColor(1.0, 0.0, 0.0);

  gui_vars.gl_segment_lcc.SetLineColor(1.0, 1.0, 0.0);

  gui_vars.gl_graph.AddChild(&gui_vars.gl_path_gt);
  gui_vars.gl_graph.AddChild(&gui_vars.gl_path_input);
  gui_vars.gl_graph.AddChild(&gui_vars.gl_path_output);
  gui_vars.gl_graph.AddChild(&gui_vars.gl_segment_lcc);


  // Add grid.
  gui_vars.gl_grid = SceneGraph::GLGrid(300, 1);
  gui_vars.gl_grid.SetVisible(false);

  // Toggle grid keyboard callback
  pangolin::RegisterKeyPressCallback('g',
                                     [&]() {
    if (gui_vars.gl_grid.IsVisible()) {
      gui_vars.gl_grid.SetVisible(false);
    }else {
      gui_vars.gl_grid.SetVisible(true);
    }
  });

  gui_vars.view_3d.SetAspect(1);

  pangolin::OpenGlRenderState stacks3d(
        pangolin::ProjectionMatrix(640, 480, 420,
                                   420, 320, 240, 1E-3, 10*1000),
        pangolin::ModelViewLookAt(5, 0, 100, 0, 0, 0, pangolin::AxisNegZ)
        );

  gui_vars.view_3d.SetHandler(
        new SceneGraph::HandlerSceneGraph(gui_vars.gl_graph, stacks3d))
      .SetDrawFunction(SceneGraph::ActivateDrawFunctor(gui_vars.gl_graph, stacks3d));

  // Add all subviews to container.
  container.AddDisplay(gui_vars.view_3d);

  std::vector<Sophus::SE3d>& path_gt_vec =
      gui_vars.gl_path_gt.GetPathRef();
  std::vector<Sophus::SE3d>& path_input_vec =
      gui_vars.gl_path_input.GetPathRef();
  std::vector<Sophus::SE3d>& path_output_vec =
      gui_vars.gl_path_output.GetPathRef();
  std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>>& segment_lcc_vec =
      gui_vars.gl_segment_lcc.GetSegmentRef();
  std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>>& pose_segment_lcc_vec =
      gui_vars.gl_segment_lcc.GetPoseSegmentRef();

  gui_vars.gl_graph.AddChild(&gui_vars.gl_grid);

  // Display the Ground Truth poses
  if(pgr->GetGroundTruth().size()){
    path_gt_vec.clear();
    for(masseuse::AbsPose pose : pgr->GetGroundTruth()){
      masseuse::Pose3 p = pose.Twp;
      path_gt_vec.push_back(p);
    }
  }

  // Dispay the initial pose values
  if(pgr->GetValues().size()){
    path_input_vec.clear();
    for(const auto& kvp : pgr->GetValues()){
      const masseuse::Pose3 p = kvp.second;
      path_input_vec.push_back(p);
    }
  }

  // Display all the loop closure constraints
  const masseuse::Graph& graph = pgr->GetGraph();
  for (const masseuse::Factor f : graph){
    if(f.isLCC){

      // Draws a line between poses that have a LCC
      masseuse::Pose3 T_a = pgr->GetValues().at(f.id1);
      masseuse::Pose3 T_b = pgr->GetValues().at(f.id2);
      segment_lcc_vec.push_back(std::make_pair(T_a, T_b));

      // Draws a line to the projected pose, where the LCC is saying the
      // other pose should be.
      const masseuse::Pose3& T_ab = f.rel_pose;
      masseuse::Pose3 T_b_lcc = T_a * T_ab;
      pose_segment_lcc_vec.push_back(std::make_pair(T_a, T_b_lcc));
    }
  }

  std::cerr << segment_lcc_vec.size() << " LCC's should be plotted" <<
               std::endl;


  while(!pangolin::ShouldQuit()) {

    if (pangolin::Pushed(ui_relax)) {
      // Reload the poses
      LoadPosesFromFile(pgr);

      // Relax the poses
      path_output_vec.clear();
      pgr->Relax();

      // Display the relaxed pose graph
      if(pgr->GetValues().size()){
        for(const auto& kvp : pgr->GetValues()){
          const masseuse::Pose3 p = kvp.second;
          path_output_vec.push_back(p);
        }
      }
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    // Toggle path visibility
    gui_vars.gl_path_gt.SetVisible(ui_show_gt_path);
    gui_vars.gl_path_input.SetVisible(ui_show_initial_path);
    gui_vars.gl_path_output.SetVisible(ui_show_relaxed_path);
    gui_vars.gl_segment_lcc.DrawSegments(ui_show_lcc_segments);
    gui_vars.gl_segment_lcc.DrawPoses(ui_show_lcc_poses);

    // Togle axis
    gui_vars.gl_path_gt.DrawAxis(draw_axis);
    gui_vars.gl_path_input.DrawAxis(draw_axis);
    gui_vars.gl_path_output.DrawAxis(draw_axis);


    pangolin::FinishFrame();
  }
}

///////////////////////////////////////////////////////////////////////////////
void LoadPosesFromFile(std::shared_ptr<masseuse::Masseuse> relaxer){
  // Load in the poses
  if(!FLAGS_in.empty()){
    relaxer->LoadPosesFromFile(FLAGS_in);
  }else{
    std::cerr << "No data provided. Please pass in the input "
              << "poses file in the --in command line arg." <<
                 std::endl;
    exit(1);
  }

}

///////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  google::ParseCommandLineFlags(&argc, &argv, true);

  masseuse::Options options;
  // initial options, can be changed via Cvars
  options.print_full_report = true;
  options.print_minimizer_progress = false;
  options.enable_prior_at_origin = true;
  options.save_initial_values_g2o = !FLAGS_g2o.empty();
  options.save_ground_truth_g2o = !FLAGS_g2o.empty();
  options.g2o_output_dir = FLAGS_g2o;
  options.save_results_binary = !FLAGS_out.empty();
  options.rel_covariance_mult = stiffness_multiplier;
  options.cov_det_thresh = cov_det_treshold;
  options.binary_output_path = FLAGS_out;
  std::shared_ptr<masseuse::Masseuse>
      relaxer(new masseuse::Masseuse(options));

  LoadPosesFromFile(relaxer);

  // Load ground truth
  if(!FLAGS_gt.empty()){
    relaxer->LoadGroundTruth(FLAGS_gt);
  }

  InitGui(relaxer);
  Run(relaxer);

//  // Save the output to file
//  if(!FLAGS_g2o.empty()){
//    relaxer->SaveResultsG2o();
//  }


  return 0;
}

