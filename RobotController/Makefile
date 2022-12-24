SOURCES = RobotController.cc ServerSocket.cc RobotStateParser.cc
OBJECTS = $(SOURCES:.cc=.o)
EXECUTABLE = RobotController.exe

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	g++ $^ -l ws2_32 -o $@

%.o: %.cc
	g++ -I./libs/json/include/ -c $< -o $@

clean:
	rm -f $(EXECUTABLE) $(OBJECTS)
