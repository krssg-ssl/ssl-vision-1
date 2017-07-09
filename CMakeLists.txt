project(ssl-vision)
cmake_minimum_required(VERSION 2.6)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()
#defines
add_definitions(-D UNIX)
add_definitions(-D LINUX)
add_definitions(-D _GNU_SOURCE)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set (QT_USE_QTNETWORK true)
set (QT_USE_QTOPENGL true)

find_package ( Qt4  4.8.6 COMPONENTS QtCore QtGui QtXml REQUIRED )

include_directories(${PROJECT_BINARY_DIR})

include ( ${QT_USE_FILE} )
message("${QT_LIBRARIES} >><><><><")
message("cmake prefix path: ${CMAKE_PREFIX_PATH}")

include(src/shared/CMakeLists.txt.inc)

include_directories(${PROJECT_SOURCE_DIR}/src/app)
include_directories(${PROJECT_SOURCE_DIR}/src/app/gui)
include_directories(${PROJECT_SOURCE_DIR}/src/app/plugins)
include_directories(${PROJECT_SOURCE_DIR}/src/app/stacks)
message("My Msg+++++++++++++++++++++++++++++++++++++++++++++++++ ${eigen3_INCLUDE_DIRS}")
include_directories(include ${catkin_INCLUDE_DIRS} ${eigen3_INCLUDE_DIRS})
set (SRCS
	src/app/capture_thread.cpp
	src/app/framedata.cpp
	src/app/main.cpp

	src/app/gui/cameracalibwidget.cpp
	src/app/gui/colorpicker.cpp
	src/app/gui/glLUTwidget.cpp
	src/app/gui/glwidget.cpp
	src/app/gui/lutwidget.cpp
	src/app/gui/mainwindow.cpp
	src/app/gui/realtimedisplaywidget.cpp
	src/app/gui/renderoptions.cpp
	src/app/gui/videowidget.cpp
	src/app/gui/jog_dial.cpp

	src/app/plugins/plugin_cameracalib.cpp
	src/app/plugins/plugin_colorcalib.cpp
	src/app/plugins/plugin_colorthreshold.cpp
	src/app/plugins/plugin_detect_balls.cpp
	src/app/plugins/plugin_detect_robots.cpp
	src/app/plugins/plugin_find_blobs.cpp
	src/app/plugins/plugin_publishgeometry.cpp
	src/app/plugins/plugin_legacypublishgeometry.cpp
	src/app/plugins/plugin_runlength_encode.cpp
	src/app/plugins/plugin_sslnetworkoutput.cpp
	src/app/plugins/plugin_legacysslnetworkoutput.cpp
	src/app/plugins/plugin_visualize.cpp
	src/app/plugins/plugin_dvr.cpp
	src/app/plugins/visionplugin.cpp

	src/app/stacks/multistack_robocup_ssl.cpp
	src/app/stacks/multivisionstack.cpp
	src/app/stacks/stack_robocup_ssl.cpp
	src/app/stacks/visionstack.cpp
)

qt4_wrap_cpp (MOC_SRCS
	src/app/capture_thread.h

	src/app/gui/cameracalibwidget.h
	src/app/gui/glLUTwidget.h
	src/app/gui/glwidget.h
	src/app/gui/lutwidget.h
	src/app/gui/mainwindow.h
	src/app/gui/videowidget.h
	src/app/gui/jog_dial.h
	src/shared/util/lut3d.h
	
	src/app/plugins/plugin_dvr.h
	src/app/plugins/plugin_publishgeometry.h
	src/app/plugins/plugin_legacypublishgeometry.h
	src/app/plugins/visionplugin.h
	
	src/app/stacks/multistack_robocup_ssl.h
)

qt4_wrap_ui (UI_SRCS
	src/app/gui/mainwindow.ui
	src/app/gui/videowidget.ui
)

qt4_add_resources(RC_SRCS 
	src/app/gui/icons/icons_gui.qrc
	${SHARED_RC}
)

## generate moc files for graphicalClient
qt4_wrap_cpp(LCLIENT_MOC_SRCS
    src/logClient/CentralWindow.h
    src/logClient/ClientThreading.h
    src/logClient/GraphicsPrimitives.h
    src/logClient/LogControl.h
)

## generate moc files for graphicalClient2
qt4_wrap_cpp(GCLIENT_MOC_SRCS
    src/graphicalClient/soccerview.h
)

## shared qt wrappings

qt4_wrap_cpp (SHARED_MOC_SRCS
	${SHARED_HEADERS}
)

qt4_add_resources(SHARED_RC_SRCS 
	${SHARED_RC}
)

set (arch ${CMAKE_SYSTEM_PROCESSOR})

message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message (STATUS "Arch: ${arch}")

if (${arch} MATCHES "x86_64")
	#set compiler flags here for x86_64
	set (cc_flags "")
elseif (${arch} MATCHES "x86")
	#set compiler flags here for x86
	set (cc_flags "")
endif()

#flags to set in debug mode
set (CMAKE_CXX_FLAGS_DEBUG "-g -Wl,--no-as-needed")

#flags to set in release mode
set (CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O3 -march=native -Wl,--no-as-needed")

message( STATUS "CC Flags: ${CMAKE_CXX_FLAGS_RELEASE}")
message (STATUS "Custom CC Flags: ${cc_flags}")

add_definitions(${cc_flags})

## build the common code
add_library(sslvision STATIC ${SHARED_MOC_SRCS} ${SHARED_RC_SRCS} ${CC_PROTO} ${SHARED_SRCS})
add_dependencies(sslvision GenerateProto)
message("<><> ${CC_PROTO}")
set (libs ${QT_LIBRARIES} dc1394 jpeg png -I/usr/local/lib/ -lprotobuf pthread GL GLU sslvision)

## build the main app
set (target vision)
add_executable(${target} ${UI_SRCS} ${MOC_SRCS} ${RC_SRCS} ${SRCS})
target_link_libraries(${target} ${libs} ${catkin_LIBRARIES} -lpthread -ldc1394 -L/usr/lib/x86_64-linux-gnu/ -lQtCore -I/home/kgpkubs/local/include -L/home/kgpkubs/local/lib -lprotobuf -lpthread -pthread)
set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

##build non graphical client
set (client client)
add_executable(${client} src/client/main.cpp )
target_link_libraries(${client} ${libs} -lpthread -ldc1394 -L/usr/lib/x86_64-linux-gnu/ -lQtCore -I/home/kgpkubs/local/include -L/home/kgpkubs/local/lib -lprotobuf -lpthread -pthread)
set_target_properties(${client} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

##build logging client
set (lclient logClient)
add_executable(${lclient} ${LCLIENT_MOC_SRCS}
	src/logClient/main.cpp 
	src/logClient/CentralWindow.cpp
	src/logClient/ClientThreading.cpp
	src/logClient/GraphicsPrimitives.cpp
	src/logClient/LogControl.cpp
)
target_link_libraries(${lclient} ${libs} -lpthread -ldc1394 -L/usr/lib/x86_64-linux-gnu/ -lQtCore -I/home/kgpkubs/local/include -L/home/kgpkubs/local/lib -lprotobuf -lpthread -pthread)
set_target_properties(${lclient} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

##build graphical client
set (gclient graphicalClient)
add_executable(${gclient} ${GCLIENT_MOC_SRCS}
  src/graphicalClient/main.cpp
  src/graphicalClient/soccerview.cpp
  src/graphicalClient/gltext.cpp
)
target_link_libraries(${gclient} ${libs} -lpthread -ldc1394 -L/usr/lib/x86_64-linux-gnu/ -lQtCore -I/home/kgpkubs/local/include -L/home/kgpkubs/local/lib -lprotobuf -lpthread -pthread)
set_target_properties(${gclient} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
