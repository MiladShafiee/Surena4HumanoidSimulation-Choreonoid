
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

option(BUILD_HELLO_WORLD_SAMPLE "Building a Hello World sample plugin" OFF)



set (My_ui  mainwindow1.ui
      )

#set (MyProjectBin_src ${PROJECT_SOURCE_DIR}/main.cpp)


qt5_wrap_ui (My_ui_moc  ${My_ui})


include_directories (${PROJECT_SOURCE_DIR})

if(BUILD_HELLO_WORLD_SAMPLE)
  set(target CnoidHelloWorldPlugin)


ADD_LIBRARY(choreonoidsurena2
 SHARED
 mainwindow1.h
 mainwindow1.cpp
 ${My_ui_moc}
)



  add_cnoid_plugin(${target} SHARED
 HelloWorldPlugin.cpp
 mainwindow1.h
 mainwindow1.cpp
#main.cpp
${My_ui_moc}
)
  target_link_libraries(choreonoidsurena2
    ${target}
    CnoidBase
     Qt5::Core
    ${Qt5Widgets_LIBRARIES}
    Qt5::PrintSupport
    ${QT_LIBRARIES}
             )
  apply_common_setting_for_plugin(${target})

  if(QT5)
    qt5_use_modules(${target}  Core Widgets  PrintSupport)
  endif()
endif()
#add_executable(MyProject main.cpp)
target_link_libraries ( choreonoidsurena2)
install_sample_source(HelloWorldPlugin.cpp main.cpp)




