
SOURCES = RobotController.cc MathUtils.cc Vision.cc CameraReceiver.cc ServerSocket.cc RobotStateParser.cc \
		Graphics\GameLoop.cc Graphics\Camera.cc Graphics\Window\Clock.cc Graphics\Window\Window.cc Graphics\Model\Object.cc Graphics\Model\OpenGLModel.cc

OBJECTS = $(SOURCES:.cc=.o)
EXECUTABLE = RobotController.exe

# Add OpenCV library paths
OPENCV_LIB_PATH = C:/opencv/build/x64/mingw/lib/
GLEW_LIB_PATH = ./libs/GLEW/lib/Release/x64/
SMFL_LIB_PATH = ./libs/SFML/lib
SOIL_LIB_PATH = ./libs/SOIL/lib/windows

# Add OpenCV include paths
OPENCV_INCLUDE_PATH = C:/opencv/build/include
GLEW_INCLUDE_PATH = ./libs/GLEW/include/
GLM_INCLUDE_PATH  = ./libs/GLM
SMFL_INCLUDE_PATH = ./libs/SFML/include
SOIL_INCLUDE_PATH = ./libs/SOIL/src/SOIL2

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	g++ $^ -L$(OPENCV_LIB_PATH) -lopencv_core455 -lopencv_imgproc455 -lopencv_highgui455 -lopencv_calib3d455 -lopencv_features2d455 \
	-L$(GLEW_LIB_PATH) -L$(SMFL_LIB_PATH) -L$(SOIL_LIB_PATH) -lsoil2-debug -lsfml-system -lsfml-window  -lglew32s -lws2_32 -lglew32 -lopengl32 \
	-lws2_32 -o $@

%.o: %.cc
	g++ -I./libs/json/include/ -I$(OPENCV_INCLUDE_PATH) \
	-I$(GLEW_INCLUDE_PATH) -I$(GLM_INCLUDE_PATH) -I$(SMFL_INCLUDE_PATH) -I$(SOIL_INCLUDE_PATH) -c $< -o $@

clean:
	del -Force $(EXECUTABLE)
	del -Force $(OBJECTS)
