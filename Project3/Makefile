#test
OMPL_DIR = /usr
CXX_FLAGS = -Wall -O2 -std=c++11 # change -02 to -g when debugging code
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app_base -lompl_app -lboost_program_options -lboost_system
CXX=c++

all: RandomTree

RandomTree: RandomTree.o main.o CollisionChecking.o
	$(CXX) $(CXXFLAGS) -o RandomTree RandomTree.o main.o CollisionChecking.o $(LD_FLAGS)

clean:
	rm -f *.o RandomTree

%.o: %.cpp
	$(CXX) -c $(CXX_FLAGS) $(INCLUDE_FLAGS) $< -o $@
