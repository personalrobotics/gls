// Standard C++ libraries
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <typeinfo>

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
#include <rsmotion/rsmotion.h>

using namespace gls::datastructures;

/// Checks for bounds only
/// This is bound to the stateValidityChecker of the ompl StateSpace.
/// \param[in] state The ompl state to check for validity.
bool isPointValid(const ompl::base::State* state, ompl::base::RealVectorBounds& bounds, std::function<std::vector<double>(std::vector<double>)> lat2real) {
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  std::vector<double> real_values = lat2real(std::vector<double>{values[0], values[1], values[2], values[3]});
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
    std::vector<double>real_vals;
    real_vals.reserve(3);
    real_vals[0] = (double)DISCXY2CONT(vals[0], mReader->resolution);
    real_vals[1] = (double)DISCXY2CONT(vals[1], mReader->resolution);
    real_vals[2] = (double) mReader->DiscTheta2ContNew(vals[2]);

    return real_vals;
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
std::vector<std::tuple<IVertex, VertexProperties, double, int>> transition_function(IVertex vi, VertexProperties vp, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){
    std::vector<std::tuple<IVertex, VertexProperties, double, int>> neighbors; 

    // Get State
    double* values = vp.getState()->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    // Neighbors
    int nx, ny, ntheta;
    IVertexHash hash;
    IVertex neighbor;
    double length; //debug
    VertexProperties neighborProperties;
    std::vector<gls::io::MotionPrimitive> mprims = mReader->mprimV;
    for (gls::io::MotionPrimitive mprim : mprims){

        // prims for current theta
        if(mprim.starttheta_c == values[2]){
            // Apply mprim
            nx = mprim.endcell.x + values[0];
            ny = mprim.endcell.y + values[1];
            ntheta = mprim.endcell.theta;

            // Length calculation
            //length = mprim.additionalactioncostmult*CONTXY2DISC(mprim.length, mReader->resolution);
            length = CONTXY2DISC(mprim.length, mReader->resolution);

            // Set state
            // TODO (schmittle) this seems inefficient to make a new state
            neighborProperties = VertexProperties();
            StatePtr newState(new State(space));
            space->copyFromReals(newState->getOMPLState(), std::vector<double>{nx, ny, ntheta, -1});
            neighborProperties.setState(newState);

            neighbors.push_back(std::tuple<IVertex, VertexProperties, double, int>(hash(neighborProperties.getState()), neighborProperties, length, mprim.motprimID));
        }
    }

    return neighbors;
}

std::vector<StatePtr> interpolate(StatePtr state, int motPrimID, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){

    double* values = 
        state->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::vector<StatePtr> states;
    double res = mReader->resolution;

    std::vector<gls::io::MotionPrimitive> mprims = mReader->mprimV;
    for(gls::io::MotionPrimitive mprim : mprims){
        if(mprim.motprimID == motPrimID && mprim.starttheta_c == values[2]){
            for(gls::io::pt_xyt point : mprim.intermptV){
                StatePtr newState(new State(space));
                space->copyFromReals(newState->getOMPLState(), 
                        std::vector<double>{(double)point.x + DISCXY2CONT(values[0], res), 
                                            (double)point.y + DISCXY2CONT(values[1], res), 
                                            (double)point.theta, -1});
                                            
                states.push_back(newState);
            }
        }
    }
    return states;
}

double heuristic(StatePtr v, StatePtr target, std::shared_ptr<ompl::base::RealVectorStateSpace> space){
    double* values = 
        v->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* goal_values = 
        target->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    return std::sqrt(std::pow(values[0]-goal_values[0],2) + std::pow(values[1]-goal_values[1], 2));

}
double rsheuristic(StatePtr v, StatePtr target, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){
    double* values = 
        v->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* goal_values = 
        target->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    using namespace rsmotion::math;

    // set the wheelbase to 0.44 meter
    const float wheelbase = 0.44f;

    // set the start position to the origin
    const Vec3f startPosition {DISCXY2CONT(values[0], mReader->resolution)
                            , 0.f, DISCXY2CONT(values[1], mReader->resolution)};

    // set the orientation (yaw; around y axis) to zero degrees (i.e. no rotation)
    const Quatf startOrientation { Vec3f{0,1,0}, Anglef::Radians(
            mReader->DiscTheta2ContNew(values[2])+1.57) };

    // create the initial CarState
    rsmotion::CarState carStart{{startPosition, startOrientation}, wheelbase};

    const Vec3f finishPosition {DISCXY2CONT(goal_values[0], mReader->resolution), 0.f, 
        DISCXY2CONT(goal_values[1], mReader->resolution)};
    const Quatf finishOrientation { Vec3f{0,1,0}, Anglef::Radians(
            mReader->DiscTheta2ContNew(goal_values[2])+1.57) };
    const rsmotion::PointState finishPoint {finishPosition, finishOrientation};

    const auto path = SearchShortestPath(carStart, finishPoint);
    //std::cout<<DISCXY2CONT(values[0], mReader->resolution)<< " "<< DISCXY2CONT(values[1], mReader->resolution)<<" "<<mReader->DiscTheta2ContNew(values[2])<<std::endl;
    //std::cout<<DISCXY2CONT(goal_values[0], mReader->resolution)<< " "<< DISCXY2CONT(goal_values[1], mReader->resolution)<<" "<<mReader->DiscTheta2ContNew(goal_values[2])<<std::endl;
    if(values[2] == 0 && values[1] == 59){
        std::cout<<values[0]<< " "<< values[1]<<" "<<values[2]<<std::endl;
        std::cout<<goal_values[0]<< " "<< goal_values[1]<<" "<<goal_values[2]<<std::endl;
        std::cout<<DISCXY2CONT(values[0], mReader->resolution) - DISCXY2CONT(goal_values[0], mReader->resolution)<<" "<<path.Length()<<" "<<CONTXY2DISC(path.Length(), mReader->resolution)<<std::endl<<std::endl;
    }
    return CONTXY2DISC(path.Length(), mReader->resolution); // slow so added weight
}

/// Displays path
/// \param[in] obstacleFile The file with obstacles stored
/// \param[in] path OMPL path
void displayPath(std::string obstacleFile, std::shared_ptr<ompl::geometric::PathGeometric> path, double resolution, gls::io::MotionPrimitiveReader *mReader) {
  // Get state count
  int pathSize = path->getStateCount();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  // TODO don't do this
  double scale = 500.0;
  int offsetx = 700;
  int offsety = -700;
  int car_int = 4;
  cv::Scalar box_color(255, 0, 0);
  auto goalState = path->getState(pathSize-1);
  double* goalVals = goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  bool cutshort = false;
  for (int i = 0; i < pathSize; ++i) {
    auto uState = path->getState(i);
    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    cv::Point uPoint((int)(u[0]*scale-offsetx), (int)(numberOfRows - u[1]*scale)-offsety);

    if (i != pathSize-1){ // this is an optimization that shouldn't just be in viz
        auto vState = path->getState(i + 1);
        double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        cv::Point vPoint((int)(v[0]*scale-offsetx), (int)(numberOfRows - v[1]*scale)-offsety);
        cv::line(image, uPoint, vPoint, cv::Scalar(0, 0, 0), 1);
        if(CONTXY2DISC(v[0],resolution) == CONTXY2DISC(goalVals[0],resolution) 
                && CONTXY2DISC(v[1],resolution) == CONTXY2DISC(goalVals[1],resolution)
                && mReader->ContTheta2DiscNew(v[2]) == mReader->ContTheta2DiscNew(goalVals[2])){
            cutshort = true;
            u = goalVals;
        }
    }


    if(i%car_int == 0 || i==pathSize-1 || cutshort){
        if(i == 0){
            box_color = cv::Scalar(0, 255, 0);
        }
        if(i == pathSize-1 || cutshort){
            box_color = cv::Scalar(0, 0, 255);
        }
        std::pair<double,double> blc = {-0.44*std::cos(u[2]) + 0.135*std::sin(u[2]), -0.44*std::sin(u[2]) - 0.135*std::cos(u[2])};
        std::pair<double,double> tlc = {-0.44*std::cos(u[2]) - 0.135*std::sin(u[2]), -0.44*std::sin(u[2]) + 0.135*std::cos(u[2])};
        std::pair<double,double> trc = {-0.135*std::sin(u[2]), 0.135*std::cos(u[2])};
        std::pair<double,double> brc = {0.135*std::sin(u[2]),-0.135*std::cos(u[2])};

        cv::Point corner1((int)((u[0]+blc.first)*scale-offsetx), (int)(numberOfRows - (u[1]+blc.second)*scale)-offsety);
        cv::Point corner2((int)((u[0]+tlc.first)*scale-offsetx), (int)(numberOfRows - (u[1]+tlc.second)*scale)-offsety);
        cv::Point corner3((int)((u[0]+trc.first)*scale-offsetx), (int)(numberOfRows - (u[1]+trc.second)*scale)-offsety);
        cv::Point corner4((int)((u[0]+brc.first)*scale-offsetx), (int)(numberOfRows - (u[1]+brc.second)*scale)-offsety);
        cv::line(image, corner1, corner2, box_color, 1);
        cv::line(image, corner1, corner4, box_color, 1);
        cv::line(image, corner3, corner2, box_color, 1);
        cv::line(image, corner3, corner4, box_color, 1);
        box_color = cv::Scalar(255, 0, 0);
    }

    cv::circle(image, uPoint, 1, cv::Scalar(255, 0, 0), cv::FILLED);
    if(cutshort){
        break;
    }
  }
  // Start/goal
  auto startState = path->getState(0);
  double* startVals = startState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  cv::Point startPoint((int)(startVals[0]*scale-offsetx), (int)(numberOfRows - startVals[1]*scale)-offsety);
  cv::circle(image, startPoint, 3, cv::Scalar(0, 255, 0), cv::FILLED);

  cv::Point goalPoint((int)(goalVals[0]*scale-offsetx), (int)(numberOfRows - goalVals[1]*scale)-offsety);
  cv::circle(image, goalPoint, 3, cv::Scalar(0, 0, 255), cv::FILLED);

  cv::imshow("Solution Path", image);
  cv::waitKey(0);
}

int main (int argc, char const *argv[]) {

  // Load Motion Primitives
  gls::io::MotionPrimitiveReader* mReader = new gls::io::MotionPrimitiveReader();
  //TODO (schmittle) don't hardcode paths
  mReader->ReadMotionPrimitives("/home/schmittle/mushr/catkin_ws/src/gls/examples/mushr.mprim");
  //mReader->ReadMotionPrimitives("/home/schmittle/Research/boxes/pysbpl/pysbpl/mprim/mushr.mprim");
  //mReader->ReadMotionPrimitives("/home/schmittle/mushr/catkin_ws/src/pushr/mprim/pushr_sandpaper.mprim");
  std::string obstacleLocation("/home/schmittle/mushr/catkin_ws/src/gls/examples/blank.png");

  // Define the state space: R^4
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(4);
  auto bounds = ompl::base::RealVectorBounds(4); 
  bounds.setLow(-100.0); // TODO set real bounds, although I don't think these are used
  bounds.setHigh(100.0);
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  std::function<std::vector<double>(std::vector<double>)> lat2real = std::bind(&lattice2real, std::placeholders::_1, mReader);
  std::function<bool(const ompl::base::State*)> isStateValid
      = std::bind(&isPointValid, std::placeholders::_1, bounds, lat2real);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  StatePtr sourceState(new State(space));
  space->copyFromReals(sourceState->getOMPLState(), std::vector<double>{2, 3, 0, -1});

  StatePtr targetState(new State(space));
  space->copyFromReals(targetState->getOMPLState(), std::vector<double>{2, 1.5, 3.14, -1});

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
                mReader),
          std::bind(&interpolate, 
              std::placeholders::_1,
              std::placeholders::_2,
              space, 
              mReader)
          );
  planner.setHeuristic(std::bind(&rsheuristic,  
                std::placeholders::_1, 
                std::placeholders::_2, 
                space, mReader));
  /*
  planner.setHeuristic(std::bind(&heuristic,  
                std::placeholders::_1, 
                std::placeholders::_2, 
                space));
                */
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
    displayPath(obstacleLocation, path, mReader->resolution, mReader); // TODO (schmittle) mReader should not need to be here
    planner.clear();
    return 0;
  }

  return 0;
}
