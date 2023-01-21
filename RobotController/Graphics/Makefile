
SOURCES = Main.cc GameLoop.cc Camera.cc Window/Clock.cc Window/Window.cc Model/Object.cc Model/OpenGLModel.cc
OBJECTS = $(SOURCES:.cc=.o)
EXECUTABLE = Graphics.exe

GLEW_LIB_PATH = C:/Dev/BattleBots/RobotController/libs/GLEW/lib/Release/x64/
SMFL_LIB_PATH = C:/Dev/Battlebots/RobotController/libs/SFML/lib
SOIL_LIB_PATH = C:\Dev\Battlebots\RobotController\libs\SOIL\lib\windows

GLEW_INCLUDE_PATH = C:/Dev/BattleBots/RobotController/libs/GLEW/include/
GLM_INCLUDE_PATH = C:/Dev/Battlebots/RobotController/libs/GLM
SMFL_INCLUDE_PATH = C:/Dev/Battlebots/RobotController/libs/SFML/include
SOIL_INCLUDE_PATH = C:/Dev/Battlebots/RobotController/libs/SOIL/src/SOIL2

all: $(EXECUTABLE)
#-L. -lsfml-audio-2 -lsfml-graphics-2 -lsfml-network-2 -lsfml-system-2 -lsfml-window-2 -lopenal32 
$(EXECUTABLE): $(OBJECTS)
	g++ $^ -L$(GLEW_LIB_PATH) -L$(SMFL_LIB_PATH) -L$(SOIL_LIB_PATH) -lsoil2-debug -lsfml-system -lsfml-window  -lglew32s -lws2_32 -lglew32 -lopengl32 -o $@

%.o: %.cc
	g++ -I$(GLEW_INCLUDE_PATH) -I$(GLM_INCLUDE_PATH) -I$(SMFL_INCLUDE_PATH) -I$(SOIL_INCLUDE_PATH) -c $< -o $@

clean:
	del -f $(EXECUTABLE) $(OBJECTS)
