
SOURCES = RobotController.cc MathUtils.cc Vision.cc CameraReceiver.cc ServerSocket.cc RobotStateParser.cc
OBJECTS = $(SOURCES:.cc=.o)
EXECUTABLE = RobotController.exe

# Add OpenCV library paths
OPENCV_LIB_PATH = C:/opencv/build/x64/mingw/lib/

# Add OpenCV include paths
OPENCV_INCLUDE_PATH = C:/opencv/build/include

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	g++ $^ -L$(OPENCV_LIB_PATH) -lopencv_core455 -lopencv_imgproc455 -lopencv_highgui455 -lopencv_calib3d455 -lopencv_features2d455 \
	-lws2_32 -o $@

%.o: %.cc
	g++ -I./libs/json/include/ -I$(OPENCV_INCLUDE_PATH) -I$(PCL_INCLUDE_PATH) -I$(BOOST_INCLUDE_PATH) -I$(EIGEN_INCLUDE_PATH) -I$(VTK_INCLUDE_PATH) -c $< -o $@

clean:
	del -f $(EXECUTABLE) $(OBJECTS)
