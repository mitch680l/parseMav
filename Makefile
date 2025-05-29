CXX       = g++
CXXFLAGS  = -std=c++17 -Wall -Wno-address-of-packed-member
INCLUDES  = -Iinc -IparseMav/inc

SRCDIR    = src
INCDIR    = inc
BINDIR    = bin
OBJDIR    = bin

# Source files grouped by target
SRC_COMMON       = $(SRCDIR)/mavlink_usart.cpp \
                   $(SRCDIR)/helper.cpp \
                   $(SRCDIR)/reader_lib.cpp
SRC_INTERPRETER  = $(SRCDIR)/main.cpp \
                   $(SRCDIR)/Vehicle.cpp \
                   $(SRCDIR)/Command.cpp \
                   $(SRCDIR)/Rover.cpp \
                   $(SRCDIR)/Plane.cpp \
                   $(SRCDIR)/Copter.cpp
SRC_SPEAKER      = $(SRCDIR)/Speaker.cpp
SRC_TEST_MAV     = $(SRCDIR)/test_mav.cpp $(SRC_COMMON)
SRC_READER       = $(SRCDIR)/reader_main.cpp $(SRC_COMMON)
SRC_CONSUMER     = $(SRCDIR)/consumer.cpp $(SRC_COMMON)

# Object files in OBJDIR
OBJ_COMMON       = $(SRC_COMMON:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJ_INTERPRETER  = $(SRC_INTERPRETER:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJ_SPEAKER      = $(SRC_SPEAKER:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJ_TEST_MAV     = $(SRC_TEST_MAV:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJ_READER       = $(SRC_READER:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJ_CONSUMER     = $(SRC_CONSUMER:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

.PHONY: all clean test interpreter speaker test_mav reader consumer
all: interpreter speaker test_mav reader consumer

test: test_mav
	./$(BINDIR)/test_mav

# Aliases to bin targets
interpreter: $(BINDIR)/interpreter
speaker:    $(BINDIR)/speaker
test_mav:   $(BINDIR)/test_mav
reader:     $(BINDIR)/reader
consumer:   $(BINDIR)/consumer

# Ensure directories exist
$(BINDIR):
	mkdir -p $(BINDIR)

# Link rules to bin/
$(BINDIR)/interpreter: $(OBJ_INTERPRETER) $(OBJ_COMMON) | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

$(BINDIR)/speaker: $(OBJ_SPEAKER) | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

$(BINDIR)/test_mav: $(OBJ_TEST_MAV) | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

$(BINDIR)/reader: $(OBJ_READER) | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -pthread -o $@ $^

$(BINDIR)/consumer: $(OBJ_CONSUMER) | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -pthread -o $@ $^

# Pattern rule for building object files into OBJDIR
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(OBJDIR)
