find_package(Pangolin 0.1 REQUIRED)
find_package(SceneGraph 0.1 REQUIRED)
find_package(Masseuse 0.1 REQUIRED)
find_package(GFlags REQUIRED)


include_directories(
    ${Pangolin_INCLUDE_DIRS}
    ${SceneGraph_INCLUDE_DIRS}
    ${Masseuse_INCLUDE_DIRS}
    ${GFlags_INCLUDE_DIRS}
    )

set( REQUIRED_LIBRARIES
    ${Pangolin_LIBRARIES}
    ${SceneGraph_LIBRARIES}
    ${Masseuse_LIBRARIES}
    ${GFlags_LIBRARIES}
    )

set(HDRS
    ../libGUI/AnalyticsView.h
    ../libGUI/ColorPalette.h
    ../libGUI/GLVarHistory.h
    ../libGUI/TimerView.h
    ../libGUI/gui_common.h
   )

set(SRCS
    masseuse_gui.cpp
    ../libGUI/AnalyticsView.cpp
    ../libGUI/ColorPalette.cpp
   )

add_executable(masseuse_gui ${HDRS} ${SRCS})

target_link_libraries( masseuse_gui
  ${REQUIRED_LIBRARIES}
  )
