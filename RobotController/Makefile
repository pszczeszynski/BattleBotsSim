# Use MSVC compiler
CC = cl.exe
CFLAGS = /std:c++17

SOURCES = RobotController.cc MathUtils.cc Vision.cc CameraReceiver.cc ServerSocket.cc RobotStateParser.cc \
		Graphics/GameLoop.cc Graphics/Camera.cc Graphics/Window/Clock.cc Graphics/Window/Window.cc Graphics/Model/Object.cc Graphics/Model/OpenGLModel.cc

OBJECTS = $(SOURCES:.cc=.obj)
LIBS = opencv_world455.lib glew32s.lib sfml-system.lib sfml-window.lib soil2-debug.lib
EXECUTABLE = RobotController.exe

# Add OpenCV library paths
OPENCV_LIB_PATH = C:/opencv/install/x64/vc16/lib
GLEW_LIB_PATH = ./libs/GLEW/lib/Release/x64
SMFL_LIB_PATH = ./libs/SFML/lib
SOIL_LIB_PATH = ./libs/SOIL/lib/windows

# Add OpenCV include paths
OPENCV_INCLUDE_PATH = C:/opencv/install/include
GLEW_INCLUDE_PATH = ./libs/GLEW/include
GLM_INCLUDE_PATH  = ./libs/GLM
SMFL_INCLUDE_PATH = ./libs/SFML/include
SOIL_INCLUDE_PATH = ./libs/SOIL/src/SOIL2
WINDOWS_KIT_INCLUDE_PATH = "C:/Program Files (x86)/Windows Kits/10/Include/10.0.18362.0/um"
WINDOWS_KIT_INCLUDE_PATH2 = "C:/Program Files (x86)/Windows Kits/10/Include/10.0.18362.0/shared"
all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) -I ./libs/json/include -I $(OPENCV_INCLUDE_PATH) -I $(GLEW_INCLUDE_PATH) -I $(GLM_INCLUDE_PATH) -I $(SMFL_INCLUDE_PATH) -I $(SOIL_INCLUDE_PATH) \
		$(OBJECTS) /link /LIBPATH:$(OPENCV_LIB_PATH) /LIBPATH:$(GLEW_LIB_PATH) /LIBPATH:$(SMFL_LIB_PATH) /LIBPATH:$(SOIL_LIB_PATH) \
		opencv_world455.lib glew32s.lib sfml-system.lib sfml-window.lib soil2-debug.lib \
		$(LIBS) \
		kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib \
		/OUT:$(EXECUTABLE)

%.obj: %.cc
	$(CC) $(CFLAGS) -I./libs/json/include -I$(OPENCV_INCLUDE_PATH) -I$(GLEW_INCLUDE_PATH) -I$(GLM_INCLUDE_PATH) -I$(SMFL_INCLUDE_PATH) -I$(SOIL_INCLUDE_PATH) \
		-I$(WINDOWS_KIT_INCLUDE_PATH) -I$(WINDOWS_KIT_INCLUDE_PATH2)\
		$< /Fo$@

clean:
	del /F $(OBJECTS)
	del /F $(EXECUTABLE)

