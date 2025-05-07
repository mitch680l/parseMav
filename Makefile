CXX = g++
CXXFLAGS = -std=c++17 -Wall

OBJS = main.o Rover.o Plane.o Copter.o

all: interpreter

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

clean:
	rm -f *.o interpreter