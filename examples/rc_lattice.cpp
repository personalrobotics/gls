// Standard C++ libraries
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <typeinfo>
#include <math.h>
#include <cmath>

// OMPL base libraries
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Custom header files
#include "gls/GLS.hpp"

using namespace gls::datastructures;
typedef std::pair<double, double> xy_pt;

/// Checks for bounds only
/// This is bound to the stateValidityChecker of the ompl StateSpace.
/// \param[in] state The ompl state to check for validity.
bool isPointValid(const ompl::base::State* state, ompl::base::RealVectorBounds& bounds, std::function<std::vector<double>(std::vector<double>)> lat2real) {
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  std::vector<double> vec_vals = std::vector<double>{values[0], values[1], values[2], values[3]};
  std::vector<double> real_values = lat2real(vec_vals);
  if(real_values[0] > bounds.low[0] && real_values[0] < bounds.high[0]){
    if(real_values[1] > bounds.low[1] && real_values[1] < bounds.high[1])
        return true;
  }
  return false;
}

// TODO (schmittle) should be able to move the fitting functions to internal
// Discretize state to graph
// This must be user-defined because it varies per state space
StatePtr fit_state2lattice(StatePtr state, gls::io::MotionPrimitiveReader *mReader,std::shared_ptr<ompl::base::RealVectorStateSpace> space){
    std::vector<double> vals;
    vals.reserve(4);
    space->copyToReals(vals, state->getOMPLState());
    vals[0] = (double)CONTXY2DISC(vals[0], mReader->resolution);
    vals[1] = (double)CONTXY2DISC(vals[1], mReader->resolution);
    vals[2] = (double) mReader->ContTheta2DiscNew(vals[2]);

    StatePtr newState(new State(space));
    space->copyFromReals(newState->getOMPLState(), vals);
    return newState;
}

// Reverse continuous state from lattice 
// This must be user-defined because it varies per state space
std::vector<double> lattice2real(std::vector<double> vals, gls::io::MotionPrimitiveReader *mReader){
    vals[0] = (double)DISCXY2CONT(vals[0], mReader->resolution);
    vals[1] = (double)DISCXY2CONT(vals[1], mReader->resolution);
    vals[2] = (double) mReader->DiscTheta2ContNew(vals[2]);

    return vals;
}

// Reverse continuous state from lattice 
// This must be user-defined because it varies per state space
StatePtr lattice2state(StatePtr state, gls::io::MotionPrimitiveReader *mReader,std::shared_ptr<ompl::base::RealVectorStateSpace> space){
    std::vector<double> vals;
    vals.reserve(4);
    space->copyToReals(vals, state->getOMPLState());
    vals = lattice2real(vals, mReader);
    StatePtr newState(new State(space));
    space->copyFromReals(newState->getOMPLState(), vals);
    return newState;
}

// Transition function from one state to another
// Each new vertex must have a unique vertex_descriptor for lookup
// The same state should ALWAYS have the same descriptor
// AKA use the state to create the descriptor
std::vector<std::tuple<IVertex, VertexProperties, double, int>> transition_function(IVertex vi, VertexProperties vp, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){
    std::vector<std::tuple<IVertex, VertexProperties, double, int>> neighbors; 

    // Get State
    double* values = vp.getState()->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    int x = (int)values[0]; 
    int y = (int)values[1]; 
    int theta = (int)values[2];
    int level = (int)values[3]; // TODO use me!

    // Neighbors
    int nx, ny, ntheta;
    IVertexHash hash;
    IVertex neighbor;
    VertexProperties neighborProperties;
    std::vector<gls::io::MotionPrimitive> mprims = mReader->mprimV;
    for (gls::io::MotionPrimitive mprim : mprims){

        // prims for current theta
        if(mprim.starttheta_c == theta){
            // Apply mprim
            nx = mprim.endcell.x + x;
            ny = mprim.endcell.y + y;
            ntheta = mprim.endcell.theta;

            // Set state
            // TODO (schmittle) this seems inefficient to make a new state
            neighborProperties = VertexProperties();
            StatePtr newState(new State(space));
            space->copyFromReals(newState->getOMPLState(), std::vector<double>{nx, ny, ntheta, -1});
            neighborProperties.setState(newState);

            neighbors.push_back(std::tuple<IVertex, VertexProperties, double, int>(hash(neighborProperties.getState()), neighborProperties, mprim.length, mprim.motprimID));
        }
    }

    return neighbors;
}

// Precompute reconfigure motion primitives using ReedShepp
std::vector<gls::io::MotionPrimitive> computeReconfigures(double block_length, double rear2block, double turning_radius){
    ompl::base::ReedsSheppStateSpace* rs_space = new ompl::base::ReedsSheppStateSpace(turning_radius);

    ompl::base::State* startState = rs_space->allocState();
    rs_space->copyFromReals(startState, std::vector<double>{0.0, 0.0, 0.0});

    ompl::base::State* rightState = rs_space->allocState();
    rs_space->copyFromReals(rightState, std::vector<double>{block_length/2.0+rear2block, -(block_length/2.0 + rear2block), M_PI/2.0});

    ompl::base::State* opState = rs_space->allocState();
    rs_space->copyFromReals(opState, std::vector<double>{block_length+2*rear2block, 0.0, M_PI});

    ompl::base::State* leftState = rs_space->allocState();
    rs_space->copyFromReals(leftState, std::vector<double>{block_length/2.0+rear2block, block_length/2.0 + rear2block, 1.5*M_PI});

    //TODO finish this. Current problem is ompl reedshepp only gives the shortest curve not all curves so if that curve is in collision you are shit out of luck
} 

// Make mushr car footprint for collision checking
std::vector<xy_pt> make_robot_footprint(double width, double length, double resolution){
    // Assuming car oriented at [0, 0, 0] and position is with respect to the rear axle
    std::vector<xy_pt> footprint;

    // We want discretization to be conservative
    double ceil_length = std::ceil(length/resolution)*resolution;
    double ceil_width = std::ceil(width/resolution)*resolution;

    double halfwidthd = (double)CONTXY2DISC(ceil_width/2.0, resolution);
    double lengthd = (double)CONTXY2DISC(ceil_length, resolution);
    // small sides
    for (double z=-halfwidthd; z <= halfwidthd; z +=1.0){
        // long sides
        for (double j=0.0; j <= lengthd; j +=1.0){
            footprint.push_back(xy_pt{j, z});
        } 
    } 

    return footprint;
}

// Make block footprint for collision checking
std::vector<xy_pt> make_block_footprint(double width, double resolution){
    // Assuming block oriented at [0, 0, 0] and cube
    
    std::vector<xy_pt> footprint;

    // We want discretization to be conservative
    double ceil_width = std::ceil(width/resolution)*resolution;

    double halfwidthd = (double)CONTXY2DISC(ceil_width/2.0, resolution);
    // small sides
    for (double z=-halfwidthd; z <= halfwidthd; z +=1.0){
        // long sides
        for (double j=-halfwidthd; j <= halfwidthd; j +=1.0){
            footprint.push_back(xy_pt{j, z});
        } 
    } 

    return footprint;
}

/// Displays path
/// \param[in] obstacleFile The file with obstacles stored
/// \param[in] path OMPL path
void displayPath(std::string obstacleFile, std::shared_ptr<ompl::geometric::PathGeometric> path) {
  // Get state count
  int pathSize = path->getStateCount();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  for (int i = 0; i < pathSize - 1; ++i) {
    auto uState = path->getState(i);
    auto vState = path->getState(i + 1);
    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    // TODO (schmittle) use intermediate points in path

    // TODO don't do this
    double scale = 1.5;
    int offsetx = -300;
    int offsety = 600;
    cv::Point uPoint((int)(u[0])*scale-offsetx, (int)((numberOfRows - u[1])*scale)-offsety);
    cv::Point vPoint((int)(v[0])*scale-offsetx, (int)((numberOfRows - v[1])*scale)-offsety);

    cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 1);
  }

  cv::imshow("Solution Path", image);
  cv::waitKey(0);
}

int main (int argc, char const *argv[]) {

  // Load Motion Primitives
  gls::io::MotionPrimitiveReader* mReader = new gls::io::MotionPrimitiveReader();
  //TODO (schmittle) don't hardcode paths
  // TODO pushr.mprim
  mReader->ReadMotionPrimitives("/home/schmittle/Research/boxes/pysbpl/pysbpl/mprim/mushr.mprim",
         "/home/schmittle/Research/boxes/pysbpl/pysbpl/mprim/mushr.json");
  std::string obstacleLocation("/home/schmittle/mushr/catkin_ws/src/gls/examples/blank.png");

  // Define the state space: R^4
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(4);
  auto bounds = ompl::base::RealVectorBounds(4); 
  bounds.setLow(-100.0); // TODO set real bounds, although I don't think these are used
  bounds.setHigh(100.0);
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // TODO (schmittle) synchronize these hard-coded measurements across all of pushr
  std::vector<xy_pt> robot_footprint = make_robot_footprint(0.27, 0.44, mReader->resolution);
  std::vector<xy_pt> block_footprint = make_block_footprint(0.1, mReader->resolution);

  std::cout<< "ROBOT"<<std::endl;
  for(xy_pt pt:robot_footprint){
      std::cout<<pt.first<<" "<<pt.second<<std::endl;
  }
  std::cout<< "BLOCK"<<std::endl;
  for(xy_pt pt:block_footprint){
      std::cout<<pt.first<<" "<<pt.second<<std::endl;
  }

  // Space Information
  std::function<std::vector<double>(std::vector<double>)> lat2real = std::bind(&lattice2real, std::placeholders::_1, mReader);
  std::function<bool(const ompl::base::State*)> isStateValid
      = std::bind(&isPointValid, std::placeholders::_1, bounds, lat2real);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  StatePtr sourceState(new State(space));
  space->copyFromReals(sourceState->getOMPLState(), std::vector<double>{20, 20, 1, -1});

  StatePtr targetState(new State(space));
  space->copyFromReals(targetState->getOMPLState(), std::vector<double>{3, 3, 2, -1});

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(sourceState->getOMPLState());
  pdef->setGoalState(targetState->getOMPLState());

  // Setup planner
  gls::GLS planner(si);

  planner.setImplicit(
          std::bind(&fit_state2lattice,  
                std::placeholders::_1, 
                mReader,
                space),
          std::bind(&transition_function, 
                std::placeholders::_1, 
                std::placeholders::_2, 
                space, 
                mReader));
  auto event = std::make_shared<gls::event::ShortestPathEvent>();
  auto selector = std::make_shared<gls::selector::ForwardSelector>();
  planner.setEvent(event);
  planner.setSelector(selector);

  planner.setup();
  planner.setProblemDefinition(pdef);

  // Solve the motion planning problem
  ompl::base::PlannerStatus status;
  status = planner.solve(ompl::base::plannerNonTerminatingCondition());

  // Obtain required data if plan was successful
  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
    // Display path and specify path size
    auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
    std::cout << "Number of Edge Evaluations: " << planner.getNumberOfEdgeEvaluations()
              << std::endl;
    displayPath(obstacleLocation, path);
    planner.clear();
    return 0;
  }

  return 0;
}
