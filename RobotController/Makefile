
SOURCES = RobotController.cc MathUtils.cc Vision.cc CameraReceiver.cc ServerSocket.cc RobotStateParser.cc
OBJECTS = $(SOURCES:.cc=.o)
EXECUTABLE = RobotController.exe

# Add OpenCV library paths
OPENCV_LIB_PATH = C:/opencv/build/x64/mingw/lib/
# Add PCL library paths (need quotes because spaces)
PCL_LIB_PATH = 'C:/Program Files/PCL 1.13.0/lib/'
BOOST_LIB_PATH = 'C:/Program Files/PCL 1.13.0/3rdParty/Boost/lib'
VTK_LIB_PATH = 'C:/Program Files/PCL 1.13.0/3rdParty/VTK/lib'

# Add OpenCV include paths
OPENCV_INCLUDE_PATH = C:/opencv/build/include
# Add PCL include paths (need quotes because spaces)
PCL_INCLUDE_PATH =   'C:/Program Files/PCL 1.13.0/include/pcl-1.13/'
BOOST_INCLUDE_PATH = 'C:/Program Files/PCL 1.13.0/3rdParty/Boost/include/boost-1_80'
EIGEN_INCLUDE_PATH = 'C:/Program Files/PCL 1.13.0/3rdParty/Eigen/eigen3'
VTK_INCLUDE_PATH =   'C:/Program Files/PCL 1.13.0/3rdParty/VTK/include/vtk-9.2'

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	g++ -v $^ -L$(OPENCV_LIB_PATH) -lopencv_core455 -lopencv_imgproc455 -lopencv_highgui455 -lopencv_calib3d455 -lopencv_features2d455 \
	-L$(PCL_LIB_PATH) -lpcl_common -lpcl_io -lpcl_filters -lpcl_kdtree -lpcl_registration -lpcl_sample_consensus -lpcl_segmentation -lpcl_visualization -lpcl_surface \
	-L$(BOOST_LIB_PATH) -L$(VTK_LIB_PATH) -lboost_system-vc143-mt-x64-1_80 -lboost_thread-vc143-mt-x64-1_80 -lvtkCommonCore-9.2 -lvtkFiltersSources-9.2 \
	-lws2_32 -o $@

%.o: %.cc
	g++ -I./libs/json/include/ -I$(OPENCV_INCLUDE_PATH) -I$(PCL_INCLUDE_PATH) -I$(BOOST_INCLUDE_PATH) -I$(EIGEN_INCLUDE_PATH) -I$(VTK_INCLUDE_PATH) -c $< -o $@

clean:
	del -f $(EXECUTABLE) $(OBJECTS)
