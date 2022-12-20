SOURCES = RobotController.cc ServerSocket.cc RobotStateParser.cc
EXECUTABLE = RobotController.exe

all: $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES)
	g++ -I./libs/json/include/ $^ -l ws2_32 -o $@

clean:
	rm -f $(EXECUTABLE)