# Makefile

CXX       = g++
CXXFLAGS  = -std=c++17 -Wall -Wno-address-of-packed-member
INCLUDES  = -I/path/to/mavlink/include

OBJS      = main.o mavlink_usart.o Rover.o Plane.o Copter.o Command.o Vehicle.o helper.o

all: interpreter speaker test_mav reader consumer

interpreter: $(OBJS)
	$(CXX) $(CXXFLAGS) -o interpreter $(OBJS)

Vehicle.o: Vehicle.cpp Vehicle.h
	$(CXX) $(CXXFLAGS) -c Vehicle.cpp

test_mav.o: test_mav.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c test_mav.cpp

test_mav: test_mav.o mavlink_usart.o
	$(CXX) $(CXXFLAGS) -o test_mav test_mav.o mavlink_usart.o helper.o

helper.o: helper.cpp helper.h
	$(CXX) $(CXXFLAGS) -c helper.cpp

main.o: main.cpp SpeechInterpreter.h
	$(CXX) $(CXXFLAGS) -c main.cpp

Command.o: Command.cpp Command.h
	$(CXX) $(CXXFLAGS) -c Command.cpp

mavlink_usart.o: mavlink_usart.cpp mavlink_usart.h
	$(CXX) $(CXXFLAGS) -c mavlink_usart.cpp

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



# reader daemon
reader: reader_main.o reader_lib.o mavlink_usart.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -pthread -o reader reader_main.o reader_lib.o mavlink_usart.o

# consumer application
consumer: consumer.o reader_lib.o mavlink_usart.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -pthread -o consumer consumer.o reader_lib.o mavlink_usart.o

# individual object rules
reader_main.o: reader_main.cpp reader.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c reader_main.cpp

reader_lib.o: reader_lib.cpp reader.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c reader_lib.cpp

consumer.o: consumer.cpp reader.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c consumer.cpp
# Run tests
.PHONY: test
test: test_heartbeat
	./test_heartbeat

clean:
	rm -f *.o interpreter speaker test_mav mavlink_uart.o