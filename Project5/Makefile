#test
OMPL_DIR = /opt/local
GO_AWAY_BOOST_WARNINGS_FLAGS = -Wno-mismatched-tags -Wno-overloaded-virtual # ignore annoying compiler warnings in boost code
CXX_FLAGS = -Wall -O2 -std=c++11 $(GO_AWAY_BOOST_WARNINGS_FLAGS) # change -02 to -g when debugging code
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app_base -lompl_app -lboost_program_options -lboost_system
CXX=clang++

all: main

main: Main.o CollisionChecking.o Needle.o SMR.o
	$(CXX) $(CXXFLAGS) -o main Main.o CollisionChecking.o Needle.o SMR.o $(LD_FLAGS)

clean:
	rm -f *.o main

%.o: %.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@
