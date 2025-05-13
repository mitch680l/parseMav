CXX = g++
CXXFLAGS = -std=c++17 -Wall

OBJS = main.o Rover.o Plane.o Copter.o

all: interpreter speaker

interpreter: $(OBJS)
	$(CXX) $(CXXFLAGS) -o interpreter $(OBJS)

main.o: main.cpp SpeechInterpreter.h
	$(CXX) $(CXXFLAGS) -c main.cpp

Rover.o: Rover.cpp Rover.h
	$(CXX) $(CXXFLAGS) -c Rover.cpp

Plane.o: Plane.cpp Plane.h
	$(CXX) $(CXXFLAGS) -c Plane.cpp

Copter.o: Copter.cpp Copter.h
	$(CXX) $(CXXFLAGS) -c Copter.cpp

Speaker.o: Speaker.cpp
	$(CXX) $(CXXFLAGS) -c Speaker.cpp

speaker: Speaker.o
	$(CXX) $(CXXFLAGS) -o speaker Speaker.o

clean:
	rm -f *.o interpreter speaker