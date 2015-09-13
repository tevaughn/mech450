OMPL_DIR = /usr
CXXFLAGS = -O2 # change to -g when debugging code
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app_base -lompl_app -lboost_program_options
CXX=c++

all:	RigidBodyPlanning RigidBodyPlanningWithControls SE2RigidBodyPlanning \
	SE3RigidBodyPlanning GeometricCarPlanning DynamicCarPlanning

clean:	
	rm -f RigidBodyPlanning RigidBodyPlanningWithControls SE2RigidBodyPlanning \
	SE3RigidBodyPlanning GeometricCarPlanning DynamicCarPlanning

RigidBodyPlanning:
	$(CXX) $(CXXFLAGS) -o RigidBodyPlanning demos/RigidBodyPlanning.cpp $(LD_FLAGS)

RigidBodyPlanningWithControls:
	$(CXX) $(CXXFLAGS) -o RigidBodyPlanningWithControls demos/RigidBodyPlanningWithControls.cpp $(LD_FLAGS)

SE2RigidBodyPlanning:
	$(CXX) $(CXXFLAGS) -o SE2RigidBodyPlanning demos/SE2RigidBodyPlanning.cpp $(LD_FLAGS)

SE3RigidBodyPlanning:
	$(CXX) $(CXXFLAGS) -o SE3RigidBodyPlanning demos/SE3RigidBodyPlanning.cpp $(LD_FLAGS)

GeometricCarPlanning:
	$(CXX) $(CXXFLAGS) -o GeometricCarPlanning demos/GeometricCarPlanning.cpp $(LD_FLAGS)

DynamicCarPlanning:
	$(CXX) $(CXXFLAGS) -o DynamicCarPlanning demos/DynamicCarPlanning.cpp $(LD_FLAGS)
