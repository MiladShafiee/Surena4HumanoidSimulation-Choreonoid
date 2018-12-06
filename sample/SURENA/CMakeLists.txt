
if(NOT ENABLE_GUI)
  return()
endif()


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# Find the QtWidgets library
find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
find_package(Qt5OpenGL)
find_package(Qt5Gui)
find_package(Qt5Core)

option(BUILD_SURENA "Building a sample plugin \"SURENA\"" OFF)
#configure_file(surena4offline.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)


set(sources
  Sources/mainwindow.cpp
 Sources//chartform.cpp
  Sources/Robot.cpp
  Sources/LinkM.cpp
  Sources/TaskSpace.cpp
  Sources/MinimumJerkInterpolation.cpp
  Sources/taskspaceoffline.cpp
  Sources/taskspaceonline.cpp
  Sources/taskspaceonline2.cpp
  Sources/qcustomplot.cpp
  Sources/taskspaceofflineRamp.cpp
)



set(headers
  Headers/chartform.h
  Headers/mainwindow.h
  Headers/Robot.h
  Headers/LinkM.h
  Headers/TaskSpace.h
  Headers/MinimumJerkInterpolation.h
  Headers/taskspaceoffline.h
  Headers/taskspaceonline.h
  Headers/taskspaceonline2.h
  Headers/qcustomplot.h
  Headers/taskspaceofflineRamp.h
)



set (MyProjectLib_ui  mainwindow.ui
      chartform.ui
      )

#set (MyProjectBin_src ${PROJECT_SOURCE_DIR}/main.cpp)


qt5_wrap_ui (MyProjectLib_ui_moc  ${MyProjectLib_ui})






if(BUILD_SURENA)
add_cnoid_simple_controller(SURENA4Offline SURENA4Offline.cpp ${sources}  ${headers})
configure_file(surena4offline.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SURENA4Online SURENA4Online.cpp ${sources} ${headers})
configure_file(surena4online.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SURENA4PDTest SURENA4PDTest.cpp ${sources} ${headers})
configure_file(surena4PDtest.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SURENA4Online1 SURENA4Online1.cpp ${sources} ${headers})
configure_file(surena4online1.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SURENA4OfflineRamp SURENA4OfflineRamp.cpp ${sources} ${headers})
configure_file(surena4offlineRamp.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SURENA4PushRecovery SURENA4PushRecovery.cpp ${sources} ${headers})
configure_file(surena4pushrecovery.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

add_cnoid_simple_controller(SURENA4Online2 SURENA4Online2.cpp ${sources} ${headers})
configure_file(surena4online2.cnoid ${CNOID_SOURCE_SHARE_DIR}/project COPYONLY)

include_directories (${PROJECT_SOURCE_DIR}/Headers)


ADD_LIBRARY(choreonoidsurena
 SHARED
 ${headers}
 ${sources}
 Sources/qcustomplot.cpp
 Headers/taskspaceoffline.h
 Headers/chartform.h
 Headers/TaskSpace.h
 Sources/chartform.cpp
 Sources/TaskSpace.cpp
 Headers/mainwindow.h
 Sources/mainwindow.cpp
  ${MyProjectLib_ui_moc}
)


  if(QT5)
    qt5_use_modules(SURENA4Offline Core Widgets  PrintSupport )
  endif()

if(QT5)
    qt5_use_modules(SURENA4Online Core  Widgets  PrintSupport )
  endif()

if(QT5)
    qt5_use_modules(SURENA4PDTest Core  Widgets  PrintSupport )
  endif()

if(QT5)
    qt5_use_modules(SURENA4Online1 Core  Widgets  PrintSupport )
  endif()

if(QT5)
    qt5_use_modules(SURENA4OfflineRamp Core  Widgets  PrintSupport )
  endif()


if(QT5)
    qt5_use_modules(SURENA4PushRecovery Core  Widgets  PrintSupport )
  endif()


if(QT5)
    qt5_use_modules(SURENA4Online2 Core  Widgets  PrintSupport )
  endif()


endif()


#qt5_wrap_cpp(plot_moc Headers/qcustomplot.h)

target_link_libraries(choreonoidsurena
    #qcustomplot
     Qt5::Core
    ${Qt5Widgets_LIBRARIES}
    Qt5::PrintSupport
    ${QT_LIBRARIES}
    CnoidBodyPlugin)

install_sample_source(sources choreonoidsurena)


