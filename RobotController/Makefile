SOURCES = RobotController.cc ServerSocket.cc
EXECUTABLE = RobotController.exe

all: $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES)
	g++ $^ -l ws2_32 -o $@

clean:
	rm -f $(EXECUTABLE)