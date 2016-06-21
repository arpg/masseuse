#pragma once
#include <vector>
#include <SceneGraph/GLDynamicGrid.h>
#include <SceneGraph/SceneGraph.h>

struct GuiVars {
  pangolin::OpenGlRenderState render_state;
  SceneGraph::GLDynamicGrid grid;
  pangolin::View* grid_view;
  pangolin::OpenGlRenderState gl_render3d;
  std::unique_ptr<SceneGraph::HandlerSceneGraph> sg_handler_;
  unsigned int panel_size = 180;

};


void InitGui(GuiVars& vars, uint32_t window_width,
                    uint32_t window_height,
                    std::string title, bool draw_panel) {

  pangolin::CreateWindowAndBind(title, window_width, window_height);

  if(draw_panel){
    pangolin::CreatePanel("ui").SetBounds(0, 1, 0,
                                          pangolin::Attach::Pix(panel_size));
  }

  vars.render_state.SetModelViewMatrix(pangolin::IdentityMatrix());
  vars.render_state.SetProjectionMatrix(pangolin::ProjectionMatrixOrthographic(
      0, window_width, 0, window_height, 0, 1000));

  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  // vars.grid.SetNumLines(20);
  // vars.grid.SetLineSpacing(5.0);
  vars.scene_graph.AddChild(&vars.grid);

  vars.grid_view = &pangolin::Display("grid")
                        .SetAspect(-(float)window_width / (float)window_height);

  vars.gl_render3d.SetProjectionMatrix(
      pangolin::ProjectionMatrix(window_width, window_height,
                                 420, 420, 320, 240, 0.01, 5000));
  vars.gl_render3d.SetModelViewMatrix(
      pangolin::ModelViewLookAt(-3, -3, -4, 0, 0, 0, pangolin::AxisNegZ));
  vars.sg_handler_.reset(new SceneGraph::HandlerSceneGraph(
      vars.scene_graph, vars.gl_render3d, pangolin::AxisNegZ, 50.0f));
  vars.grid_view->SetHandler(vars.sg_handler_.get());
  vars.grid_view->SetDrawFunction(
      SceneGraph::ActivateDrawFunctor(vars.scene_graph, vars.gl_render3d));

  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0.0, 0.0, 0.0, 1.0);

}
