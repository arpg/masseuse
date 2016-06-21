#include <iostream>

#include <masseuse/masseuse.h>
#include <glog/logging.h>

/*-----GUI Includes-----------*/
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include "../libGUI/AnalyticsView.h"
#include "../libGUI/GLPathRel.h"
#include "../libGUI/GLPathAbs.h"
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
  GLPathAbs gl_path_lcc;
  SceneGraph::GLGrid gl_grid;
};

GuiVars gui_vars;

///////////////////////////////////////////////////////////////////////////////
void InitGui(const std::shared_ptr<masseuse::Masseuse>
             pgr){


}

///////////////////////////////////////////////////////////////////////////////
void Run(const std::shared_ptr<masseuse::Masseuse>
         pgr){

  pangolin::CreateWindowAndBind("Masseuse", 1600, 800);

  // Set up panel.
  const unsigned int panel_size = 180;

  pangolin::CreatePanel("ui").SetBounds(0, 1, 0,
                                        pangolin::Attach::Pix(panel_size));

  pangolin::Var<bool>           ui_show_gt_path("ui.Show GT Path", true, true);
  pangolin::Var<bool>           ui_show_initial_path("ui.Show Initial Path", true, true);
  pangolin::Var<bool>           ui_show_relaxed_path("ui.Show Relaxed Path", true, true);
  pangolin::Var<bool>           ui_show_lcc("ui.Show Loop Closures", true, true);
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

  gui_vars.gl_path_lcc.SetPoseDisplay(0);
  gui_vars.gl_path_lcc.SetLineColor(0.0, 1.0, 1.0);

  gui_vars.gl_graph.AddChild(&gui_vars.gl_path_gt);
  gui_vars.gl_graph.AddChild(&gui_vars.gl_path_input);
  gui_vars.gl_graph.AddChild(&gui_vars.gl_path_output);
  gui_vars.gl_graph.AddChild(&gui_vars.gl_path_lcc);


  // Add grid.
  gui_vars.gl_grid = SceneGraph::GLGrid(300, 1);

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
  std::vector<Sophus::SE3d>& path_lcc_vec =
      gui_vars.gl_path_lcc.GetPathRef();

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

//  // Display all the loop closure constraints
//  const masseuse::GraphAndValues& graphAndValues =
//      pgr->GetGraphAndValues();
//  for(const masseuse::Factor f : *graphAndValues.second){
//    if(f.isLCC){
//      masseuse::Pose3 T_a = graphAndValues.first->at(f.id1);
//      masseuse::Pose3 T_b = graphAndValues.first->at(f.id2);

//    }
//  }

  while(!pangolin::ShouldQuit()) {

    if (pangolin::Pushed(ui_relax)) {
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

    gui_vars.gl_path_gt.SetVisible(ui_show_gt_path);
    gui_vars.gl_path_input.SetVisible(ui_show_initial_path);
    gui_vars.gl_path_output.SetVisible(ui_show_relaxed_path);
    gui_vars.gl_path_lcc.SetVisible(ui_show_lcc);


    pangolin::FinishFrame();
  }
}


///////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  google::ParseCommandLineFlags(&argc, &argv, true);

  masseuse::Options options;
  options.print_initial_final_error = true;
  options.save_initial_values_g2o = !FLAGS_g2o.empty();
  options.save_ground_truth_g2o = !FLAGS_g2o.empty();
  options.g2o_output_dir = FLAGS_g2o;
  options.save_results_binary = !FLAGS_out.empty();
  options.rel_covariance_mult = 1;
  options.cov_det_thresh = 1e-38;
  options.binary_output_path = FLAGS_out;
  std::shared_ptr<masseuse::Masseuse>
      relaxer(new masseuse::Masseuse(options));

  // Load in the poses
  if(!FLAGS_in.empty()){
    relaxer->LoadPosesFromFile(FLAGS_in);
  }else{
    std::cerr << "No data provided. Please pass in the input "
              << "poses file in the --in command line arg." <<
                 std::endl;
    exit(1);
  }

  // Load ground truth
  if(!FLAGS_gt.empty()){
    relaxer->LoadGroundTruth(FLAGS_gt);
  }

  InitGui(relaxer);
  std::cerr << "finished initializing gui" << std::endl;
  Run(relaxer);

//  // Save the output to file
//  if(!FLAGS_g2o.empty()){
//    relaxer->SaveResultsG2o();
//  }


  return 0;
}

