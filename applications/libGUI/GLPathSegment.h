#pragma once

#include <Eigen/Eigen>
#include <sophus/se3.hpp>

#include <SceneGraph/GLObject.h>

#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<float,4,4,Eigen::ColMajor>(m).data())


/////////////////////////////////////////////////////////////////////////////
// Code to render path segments
class GLPathSegment : public SceneGraph::GLObject
{
public:
  GLPathSegment()
  {
    m_bInitGLComplete = false;
    m_fLineColor(0) = 1.0;
    m_fLineColor(1) = 1.0;
    m_fLineColor(2) = 0.0;
    m_fLineColor(3) = 1.0;
  }

  ~GLPathSegment()
  {

  }

  // just draw the segments
  void DrawCanonicalObject()
  {
    pangolin::GlState state;

    glLineWidth(1.0f);

    if( !m_vSegments.empty() ) {

      Eigen::Matrix4f  fPose;

      if(m_bDrawSegments){
        glPushMatrix();
        glEnable( GL_LINE_SMOOTH );
        state.glLineWidth(1.5f);
        glColor4f( m_fLineColor(0), m_fLineColor(1), m_fLineColor(2), m_fLineColor(3) );

        for(auto& pair : m_vSegments){
          glBegin( GL_LINE_STRIP );

          // First pose
          Sophus::SE3d& Pose1 = pair.first;
          fPose = Pose1.matrix().cast<float>();
          glVertex3f( fPose(0,3), fPose(1,3), fPose(2,3) );

          // Second pose
          Sophus::SE3d& Pose2 = pair.second;
          fPose = Pose2.matrix().cast<float>();
          glVertex3f( fPose(0,3), fPose(1,3), fPose(2,3) );


          glEnd();
        }
        glPopMatrix();
      }

      if(m_bDrawPoses){


        glEnable( GL_LINE_SMOOTH );

        for( auto& pair : m_vPoseSegments ) {

          glPushMatrix();
          // First, draw a dotted line segment from pose a to pose b_lcc
          glPushAttrib(GL_ENABLE_BIT);
          state.glLineWidth(1.5f);
          glColor4f( m_fLineColor(0), m_fLineColor(1), m_fLineColor(2), m_fLineColor(3) );
          glLineStipple(4, 0xAAAA);
          glEnable(GL_LINE_STIPPLE);
          glBegin( GL_LINES );

          Sophus::SE3d& Pose1 = pair.first;
          Sophus::SE3d& Pose2 = pair.second;
          fPose = Pose1.matrix().cast<float>();
          glVertex3f( fPose(0,3), fPose(1,3), fPose(2,3) );

          fPose = Pose2.matrix().cast<float>();
          glVertex3f( fPose(0,3), fPose(1,3), fPose(2,3) );

          glEnd();
          glPopAttrib();
          glPopMatrix();


          glPushMatrix();

          // now draw pose b_lcc (with axis)
          glMultMatrixf(MAT4_COL_MAJOR_DATA( fPose ));

          // plot the pose
          glColor3f(1.0,0.0,0.0);
          pangolin::glDrawLine(0,0,0,1,0,0);
          glColor3f(0.0,1.0,0.0);
          pangolin::glDrawLine(0,0,0,0,-1,0);
          glColor3f(0.0,0.0,1.0);
          pangolin::glDrawLine(0,0,0,0,0,-1);
          glPopMatrix();

        }

      }

    }
  }

  std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>>& GetSegmentRef()
  {
    return m_vSegments;
  }

  std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>>& GetPoseSegmentRef()
  {
    return m_vPoseSegments;
  }

  void DrawPoses( bool Val )
  {
    m_bDrawPoses = Val;
  }

  void DrawIndices( bool Val )
  {
    m_bDrawIndices = Val;
  }

  void DrawSegments( bool Val )
  {
    m_bDrawSegments = Val;
  }


  void SetLineColor( float R, float G, float B, float A = 1.0 )
  {
    m_fLineColor(0) = R;
    m_fLineColor(1) = G;
    m_fLineColor(2) = B;
    m_fLineColor(3) = A;
  }

private:
  bool                            m_bInitGLComplete;
  bool                            m_bDrawPoses;
  bool                            m_bDrawSegments;
  bool                            m_bDrawIndices;
  Eigen::Vector4f                 m_fLineColor;
  std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>>   m_vSegments;
  std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>>   m_vPoseSegments;
};
