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
#include "dubins.h"

using namespace gls::datastructures;
typedef std::pair<double, double> xy_pt;
float WHEELBASE = 0.44;
float TURNING_RADIUS = 0.627;
int NUMTHETADIRS = 16;

// Make mushr car footprint for collision checking
std::vector<xy_pt> make_robot_footprint(double width, double length, double resolution){
    // Assuming car oriented at [0, 0, 0] and position is with respect to the rear axle
    std::vector<xy_pt> footprint;

    // We want discretization to be conservative
    //double ceil_length = std::ceil(length/resolution)*resolution;
    //double ceil_width = std::ceil(width/resolution)*resolution;
    double ceil_length = length;
    double ceil_width = width;

    double halfwidthd = std::floor(ceil_width/(2.0* resolution));
    double lengthd = std::floor(ceil_length/ resolution);
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
    //double ceil_width = std::ceil(width/resolution)*resolution;
    double ceil_width = width;

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

// Translate and rotate footprint around given position
// Assumes footprint and position are on lattice already 
std::vector<xy_pt> positionFootprint(std::vector<double> position, std::vector<xy_pt> footprint, double resolution){
    double angle = gls::io::DiscTheta2Cont((int)position[2], NUMTHETADIRS);
    std::cout<<angle<<std::endl;
    double x, y, x_rot, y_rot;
    std::vector<xy_pt> rotated_footprint;

    for(xy_pt point : footprint){

        // Convert to real and Rotate
        x = DISCXY2CONT(point.first, resolution);
        y = DISCXY2CONT(point.second, resolution);
        x_rot = x*std::cos(angle) - y*std::sin(angle);
        y_rot = x*std::sin(angle) + y*std::cos(angle);
        point.first = CONTXY2DISC(x_rot, resolution);
        point.second = CONTXY2DISC(y_rot, resolution);

        // Transform
        point.first += position[0];
        point.second += position[1];
        rotated_footprint.push_back(point);
    }
    return rotated_footprint;
}


/// Checks for bounds only
/// This is bound to the stateValidityChecker of the ompl StateSpace.
/// \param[in] state The ompl state to check for validity.
bool isPointValid(
        std::vector<double> values, 
        ompl::base::RealVectorBounds& bounds, 
        std::function<std::vector<double>(std::vector<double>)> lat2real) {
  std::vector<double> real_values = lat2real(std::vector<double>{values[0], values[1], values[2], values[3]});
  if(real_values[0] > bounds.low[0] && real_values[0] < bounds.high[0]){
    if(real_values[1] > bounds.low[1] && real_values[1] < bounds.high[1])
        return true;
  }
  return false;
}

// State wrapper
bool isStatePointValid(
        const ompl::base::State* state, 
        ompl::base::RealVectorBounds& bounds, 
        std::function<std::vector<double>(std::vector<double>)> lat2real) {
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  return isPointValid({values[0], values[1], values[2], values[3], values[4], values[5], values[6]}, bounds, lat2real);
}

// Checks if robot is 1) in bounds 2) not colliding with block
bool isRobotValid(
        std::vector<double> values, 
        ompl::base::RealVectorBounds& bounds, 
        std::function<std::vector<double>(std::vector<double>)> lat2real,
        std::vector<xy_pt> robot_footprint,
        std::vector<xy_pt> block_footprint,
        double resolution) {
  // Bounds check
  if (!isPointValid(values, bounds, lat2real)){
    return false;
  }

  std::vector<xy_pt> rrobot_fp = 
      positionFootprint({values[0], values[1], values[2]}, robot_footprint, resolution);
  std::vector<xy_pt> rblock_fp = 
      positionFootprint({values[4], values[5], values[6]}, block_footprint, resolution);

  for(xy_pt robot_point : rrobot_fp){
      if(std::find(rblock_fp.begin(), rblock_fp.end(), robot_point) != rblock_fp.end()) {
          return false;
      }
  }
  return true;
}

// State wrapper around isRobotValid
bool isRobotStateValid(
        const ompl::base::State* state, 
        ompl::base::RealVectorBounds& bounds, 
        std::function<std::vector<double>(std::vector<double>)> lat2real,
        std::vector<xy_pt> robot_footprint,
        std::vector<xy_pt> block_footprint,
        double resolution) {
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  return isRobotValid({values[0], values[1], values[2], values[3], values[4], values[5], values[6]}, bounds, lat2real, robot_footprint, block_footprint, resolution);
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

            // Length calculation TODO (schmittle) revert to use cost mul
            //length = mprim.additionalactioncostmult*CONTXY2DISC(mprim.length, mReader->resolution);
            length = CONTXY2DISC(mprim.length, mReader->resolution);
            //std::cout<<mprim.length<<" "<<length<<std::endl;

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

std::vector<StatePtr> interpolate(StatePtr state, StatePtr state2, int motPrimID, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){

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

double rsheuristic(StatePtr v, StatePtr target, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){
    double* values = 
        v->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* goal_values = 
        target->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    using namespace rsmotion::math;

    // set the wheelbase to 0.44 meter
    const float wheelbase = 0.44f;
    double turning_radius = 0.627;  // From mushr.json. TODO (schmittle) double check this matches real

    // set the start position to the origin
    const Vec3f startPosition {DISCXY2CONT(values[1], mReader->resolution)
                            , 0.f, DISCXY2CONT(values[0], mReader->resolution)};

    // set the orientation (yaw; around y axis) to zero degrees (i.e. no rotation)
    const Quatf startOrientation { Vec3f{0,1,0}, Anglef::Radians(
            mReader->DiscTheta2ContNew(values[2])) };

    // create the initial CarState
    rsmotion::CarState carStart{{startPosition, startOrientation}, wheelbase};

    const Vec3f finishPosition {DISCXY2CONT(goal_values[1], mReader->resolution), 0.f, 
        DISCXY2CONT(goal_values[0], mReader->resolution)};
    const Quatf finishOrientation { Vec3f{0,1,0}, Anglef::Radians(
            mReader->DiscTheta2ContNew(goal_values[2])) };
    const rsmotion::PointState finishPoint {finishPosition, finishOrientation};

    const auto path = SearchShortestPath(carStart, finishPoint, turning_radius);

    //std::cout<<path.Length(turning_radius)<<" "<<CONTXY2DISC(path.Length(turning_radius), mReader->resolution)<<std::endl;
    return CONTXY2DISC(path.Length(turning_radius), mReader->resolution);
}

std::pair<double, std::vector<rsmotion::algorithm::State>> testheuristic(std::vector<double> start, std::vector<double> goal){
    using namespace rsmotion::math;

    // set the wheelbase to 0.44 meter
    const float wheelbase = 0.44f;
    double turning_radius = 0.627;  // From mushr.json. TODO (schmittle) double check this matches real

    // set the start position to the origin
    const Vec3f startPosition {start[1], 0.f, start[0]};

    // set the orientation (yaw; around y axis) to zero degrees (i.e. no rotation)
    const Quatf startOrientation { Vec3f{0,1,0}, Anglef::Radians(start[2]) };

    // create the initial CarState
    rsmotion::CarState carStart{{startPosition, startOrientation}, wheelbase};

    const Vec3f finishPosition {goal[1], 0.f, goal[0]};
    const Quatf finishOrientation { Vec3f{0,1,0}, Anglef::Radians(goal[2]) };
    const rsmotion::PointState finishPoint {finishPosition, finishOrientation};

    const auto path = SearchShortestPath(carStart, finishPoint);
    const auto path2 = SearchShortestPath(carStart, finishPoint, turning_radius);
    //std::cout<<start[0]<< " "<<start[1]<<" "<<start[2]<<" || "<<goal[0]<<" "<<goal[1]<<" "<<goal[2]<<std::endl;

    auto path2_vals = rsmotion::GetPath(carStart, path2, 0.05, turning_radius);
    /*
    std::cout<<"rad "<<path2.Length(turning_radius)<<std::endl;
    for (rsmotion::algorithm::State s : path2_vals){
        std::cout<<s.X()<< " "<<s.Y()<<" "<<s.Phi()<<std::endl;
    }
    std::cout<<"path_size: "<<path2_vals.size()<<std::endl;
    auto end = rsmotion::TraversePathNormalized(1.0, path, carStart).Rear.Pos;
    std::cout<<end[0]<<" "<<end[2]<<std::endl;
    */
    return {path.Length(), path2_vals};
}

/*
void makeBox(cv::Mat image, cv::Scalar box_color, std::vector<double> u, double scale, double offsetx,double offsety, int numberOfRows){
    std::pair<double,double> blc = {0.44*std::cos(u[2]) + 0.135*std::sin(u[2]), 0.44*std::sin(u[2]) - 0.135*std::cos(u[2])};
    std::pair<double,double> tlc = {0.44*std::cos(u[2]) - 0.135*std::sin(u[2]), 0.44*std::sin(u[2]) + 0.135*std::cos(u[2])};
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
}
*/
void makeBox(cv::Mat image, 
        cv::Scalar box_color, 
        std::vector<double> position, 
        std::vector<double> dimentions, // {length, width}
        bool center, // if true consider position center of box, else rear axle
        double scale, 
        double offsetx,
        double offsety, 
        int numberOfRows){
    double offcenter = 0.0;

    if (center){
        dimentions[0] /= 2.0;
        offcenter = dimentions[0];
    }
    std::pair<double,double> blc = {dimentions[0]*std::cos(position[2]) + dimentions[1]/2.0*std::sin(position[2]), dimentions[0]*std::sin(position[2]) - dimentions[1]/2.0*std::cos(position[2])};
    std::pair<double,double> tlc = {dimentions[0]*std::cos(position[2]) - dimentions[1]/2.0*std::sin(position[2]), dimentions[0]*std::sin(position[2]) + dimentions[1]/2.0*std::cos(position[2])};

    std::pair<double,double> brc = {-offcenter*std::cos(position[2]) + dimentions[1]/2.0*std::sin(position[2]), -offcenter*std::sin(position[2]) - dimentions[1]/2.0*std::cos(position[2])};
    std::pair<double,double> trc = {-offcenter*std::cos(position[2]) - dimentions[1]/2.0*std::sin(position[2]), -offcenter*std::sin(position[2]) + dimentions[1]/2.0*std::cos(position[2])};

    cv::Point corner1((int)((position[0]+blc.first)*scale-offsetx), (int)(numberOfRows - (position[1]+blc.second)*scale)-offsety);
    cv::Point corner2((int)((position[0]+tlc.first)*scale-offsetx), (int)(numberOfRows - (position[1]+tlc.second)*scale)-offsety);
    cv::Point corner3((int)((position[0]+trc.first)*scale-offsetx), (int)(numberOfRows - (position[1]+trc.second)*scale)-offsety);
    cv::Point corner4((int)((position[0]+brc.first)*scale-offsetx), (int)(numberOfRows - (position[1]+brc.second)*scale)-offsety);
    cv::line(image, corner1, corner2, box_color, 1);
    cv::line(image, corner1, corner4, box_color, 1);
    cv::line(image, corner3, corner2, box_color, 1);
    cv::line(image, corner3, corner4, box_color, 1);
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
  int offsetx = 500;
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
    std::vector<double> uvec = {u[0], u[1], u[2], u[3]};

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
        makeBox(image, box_color, uvec, {0.44, 0.27}, false, scale, offsetx, offsety, numberOfRows);
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

void displayPath(std::string obstacleFile, std::vector<rsmotion::algorithm::State> path) {
  // Get state count
  int pathSize = path.size();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  // TODO don't do this
  double scale = 350.0;
  int offsetx = 380;
  int offsety = -500;
  int car_int = 4;
  cv::Scalar box_color(255, 0, 0);
  for (int i = 0; i < pathSize; ++i) {
    auto u = path[i];
    std::vector<double> uvec = {u.X(), u.Y(), u.Phi()};
    cv::Point uPoint((int)(u.X()*scale-offsetx), (int)(numberOfRows - u.Y()*scale)-offsety);


    if(i%car_int == 0 || i==pathSize-1){
        if(i == 0){
            box_color = cv::Scalar(0, 255, 0);
        }
        if(i == pathSize-1){
            box_color = cv::Scalar(0, 0, 255);
        }
        makeBox(image, box_color, uvec, {0.44, 0.27}, false, scale, offsetx, offsety, numberOfRows);
        box_color = cv::Scalar(255, 0, 0);
    }

    cv::circle(image, uPoint, 1, cv::Scalar(255, 0, 0), cv::FILLED);
  }
  // Start/goal
  auto startVals = path[0];
  auto goalVals = path.back();
  cv::Point startPoint((int)(startVals.X()*scale-offsetx), (int)(numberOfRows - startVals.Y()*scale)-offsety);
  cv::circle(image, startPoint, 3, cv::Scalar(0, 255, 0), cv::FILLED);

  cv::Point goalPoint((int)(goalVals.X()*scale-offsetx), (int)(numberOfRows - goalVals.Y()*scale)-offsety);
  cv::circle(image, goalPoint, 3, cv::Scalar(0, 0, 255), cv::FILLED);

  cv::imshow("Solution Path", image);
  cv::waitKey(0);
}

void displayStatePaths(std::string obstacleFile, std::vector<std::vector<rsmotion::algorithm::State>> paths, std::function<std::vector<double>(std::vector<double>)> lat2real){

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  // TODO don't do this
  double scale = 350.0;
  int offsetx = 380;
  int offsety = -150;

  // debug collision checking
  double resolution = 0.05;
  std::vector<xy_pt> robot_footprint = make_robot_footprint(0.27, 0.44, resolution);
  std::vector<xy_pt> block_footprint = make_block_footprint(0.1, resolution);
  auto bounds = ompl::base::RealVectorBounds(7); 
  bounds.setLow(-100.0);
  bounds.setHigh(100.0);
  int pathidx = 0;//debug
  for (std::vector<rsmotion::algorithm::State> path : paths){

      cv::Scalar box_color(0, 255, 0);
      cv::Scalar color(255, 0, 0);

      // Get state count
      int pathSize = path.size();


      for (int i = 0; i < pathSize; ++i) {
        auto u = path[i];
        std::vector<double> uvec = {u.X(), u.Y(), u.Phi()};

        // Check for collisions
        /*
        std::vector<double> colvec = {CONTXY2DISC(u.X(), resolution), 
                                    CONTXY2DISC(u.Y(), resolution), 
                                    gls::io::ContTheta2Disc(u.Phi(), NUMTHETADIRS), 
                                    1,
                                    40,
                                    40,
                                    0};
                                    */
        std::vector<double> colvec = {round(u.X()/ resolution), 
                                    round(u.Y()/ resolution), 
                                    gls::io::ContTheta2Disc(u.Phi(), NUMTHETADIRS), 
                                    1,
                                    40,
                                    40,
                                    0};
        if(!isRobotValid(colvec, bounds, lat2real, robot_footprint, block_footprint, resolution)){
            std::vector<xy_pt> rrobot_fp = 
              positionFootprint({CONTXY2DISC(u.X(), resolution), 
                                 CONTXY2DISC(u.Y(), resolution), 
                                 gls::io::ContTheta2Disc(u.Phi(), NUMTHETADIRS)}, 
                                 robot_footprint, resolution);

            color = cv::Scalar(0,0,255);
            for (xy_pt rpoint : rrobot_fp){
                cv::Point rPoint((int)(rpoint.first*resolution*scale-offsetx), (int)(numberOfRows - rpoint.second*resolution*scale)-offsety);
                cv::circle(image, rPoint, 1, color, cv::FILLED);
            }
            //makeBox(image, color, uvec, scale, offsetx, offsety, numberOfRows);
        }
        if(colvec[0] == 40 && colvec[1] == 40 && colvec[2] == 0){
            std::vector<xy_pt> rrobot_fp = 
              positionFootprint({std::round(u.X()/ resolution), 
                                 std::round(u.Y()/ resolution), 
                                 gls::io::ContTheta2Disc(u.Phi(), NUMTHETADIRS)}, 
                                 robot_footprint, resolution);
            for (xy_pt rpoint : rrobot_fp){
                cv::Point rPoint((int)((rpoint.first*resolution)*scale-offsetx), (int)(numberOfRows - DISCXY2CONT(rpoint.second,resolution)*scale)-offsety);
                cv::circle(image, rPoint, 2, color, cv::FILLED);
            }
            makeBox(image, color, uvec, {0.44, 0.27}, false, scale, offsetx, offsety, numberOfRows);
        }
        cv::Point uPoint((int)(u.X()*scale-offsetx), (int)(numberOfRows - u.Y()*scale)-offsety);
        if (i ==0){
            makeBox(image, box_color, uvec, {0.44, 0.27}, false, scale, offsetx, offsety, numberOfRows);
        }
        if(i == pathSize-1){
            box_color = cv::Scalar(0, 0, 255);
            makeBox(image, box_color, uvec, {0.44, 0.27}, false, scale, offsetx, offsety, numberOfRows);
        }
        cv::circle(image, uPoint, 1, color, cv::FILLED);
        color = cv::Scalar(255, 0, 0);
      }
      pathidx++;//debug
  }
  cv::Point bPoint((int)(49*resolution*scale-offsetx), (int)(numberOfRows - 26*resolution*scale)-offsety);
  cv::circle(image, bPoint, 3, cv::Scalar(255,255,0), cv::FILLED);

  std::vector<rsmotion::algorithm::State> path = paths.back();
  // Start/goal
  auto startVals = path[0];
  auto goalVals = path.back();
  cv::Point startPoint((int)(startVals.X()*scale-offsetx), (int)(numberOfRows - startVals.Y()*scale)-offsety);
  cv::circle(image, startPoint, 3, cv::Scalar(0, 255, 0), cv::FILLED);

  cv::Point goalPoint((int)(goalVals.X()*scale-offsetx), (int)(numberOfRows - goalVals.Y()*scale)-offsety);
  cv::circle(image, goalPoint, 3, cv::Scalar(0, 0, 255), cv::FILLED);

  cv::imshow("All Paths", image);
  cv::waitKey(0);
}

void displayPoints(cv::Mat image, std::vector<xy_pt> points, double resolution, cv::Scalar color) {
  // Get state count

  // Obtain the image matrix
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  // TODO don't do this
  double scale = 500.0;
  int offsetx = -300;
  int offsety = 400;
  int thickness = 1;
  for (xy_pt point : points){
      if(point.first == 0 && point.second == 0){
          thickness = 3;
      }
    cv::Point uPoint((int)
            (DISCXY2CONT(point.first, resolution)*scale-offsetx), 
            (int)(numberOfRows - DISCXY2CONT(point.second, resolution)*scale)-offsety);

    cv::circle(image, uPoint, thickness, color, cv::FILLED);
    thickness = 1;
  }

}

void displayPaths(std::string obstacleFile, std::vector<rsmotion::algorithm::Path> paths,rsmotion::CarState carStart, double resolution, double turning_radius, std::function<std::vector<double>(std::vector<double>)> lat2real) {

  std::vector<std::vector<rsmotion::algorithm::State>> state_paths;
  for (rsmotion::algorithm::Path p : paths){
      state_paths.push_back(rsmotion::GetPath(carStart, p, resolution, turning_radius));
  }
  displayStatePaths(obstacleFile, state_paths, lat2real);
}

int savePath(double q[3], double x, void* user_data, std::vector<rsmotion::algorithm::State>* saved_path) {
    rsmotion::algorithm::State dstate(q[0], q[1], q[2]-M_PI, false);
    saved_path->push_back(dstate);
    return 0;
}


int main (int argc, char const *argv[]) {

  // Load Motion Primitives
  gls::io::MotionPrimitiveReader* mReader = new gls::io::MotionPrimitiveReader();
  //TODO (schmittle) don't hardcode paths
  mReader->ReadMotionPrimitives("/home/schmittle/mushr/catkin_ws/src/gls/examples/mushr.mprim",
          "/home/schmittle/mushr/catkin_ws/src/gls/examples/mushr.json");
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
      = std::bind(&isStatePointValid, std::placeholders::_1, bounds, lat2real);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  StatePtr sourceState(new State(space));
  space->copyFromReals(sourceState->getOMPLState(), std::vector<double>{3, 3, 0, -1});

  StatePtr targetState(new State(space));
  space->copyFromReals(targetState->getOMPLState(), std::vector<double>{2, 3, 0, -1});

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
              std::placeholders::_3,
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

  /*
  // Solve the motion planning problem
  ompl::base::PlannerStatus status;
  status = planner.solve(ompl::base::plannerNonTerminatingCondition());

  // Obtain required data if plan was successful
  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
    // Display path and specify path size
    auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
    std::cout<<"Path length: "<< path->length()<<std::endl;
    std::cout << "Number of Edge Evaluations: " << planner.getNumberOfEdgeEvaluations()
              << std::endl;
    displayPath(obstacleLocation, path, mReader->resolution, mReader); // TODO (schmittle) mReader should not need to be here
    planner.clear();
  }
  */

  /* debug stuff*/

  double resolution = mReader->resolution;
  std::vector<xy_pt> robot_footprint = make_robot_footprint(0.27, 0.44, resolution);
  std::vector<xy_pt> block_footprint = make_block_footprint(0.1, resolution);

  auto rotated_footprint = positionFootprint({0, 0, 4}, robot_footprint, 0.05);
  auto rotated_block_footprint = positionFootprint({0,CONTXY2DISC(0.05+WHEELBASE, resolution), 0}, block_footprint, 0.05);
  cv::Mat image = cv::imread(obstacleLocation, 1);

  //displayPoints(image, robot_footprint, 0.05, cv::Scalar(255,0,0));
  displayPoints(image, rotated_footprint, 0.05, cv::Scalar(0,0,255));
  displayPoints(image, rotated_block_footprint, 0.05, cv::Scalar(0,0,255));
  double scale = 500.0;
  int offsetx = -300;
  int offsety = 400;
  int numberOfRows = image.rows;
  std::vector<double> uvec = {0, 0, 0};
  makeBox(image, cv::Scalar(0,255,0), uvec, {WHEELBASE, 0.27}, false, scale, offsetx, offsety, numberOfRows);
  makeBox(image, cv::Scalar(0,255,0), {0.05 + WHEELBASE,0,0}, {0.1, 0.1}, true, scale, offsetx, offsety, numberOfRows);
  cv::imshow("Points", image);
  cv::waitKey(0);

  // Reeds Shepp
  /*
  auto rsspace = std::make_shared<ompl::base::ReedsSheppStateSpace>(0.627);
  auto rsbounds = ompl::base::RealVectorBounds(2); 
  rsbounds.setLow(-100.0); // TODO set real bounds, although I don't think these are used
  rsbounds.setHigh(100.0);
  rsspace->setBounds(rsbounds);
  rsspace->setLongestValidSegmentFraction(0.1 / rsspace->getMaximumExtent());
  rsspace->setup();

  std::vector<double> startvect;
  std::vector<double> goalvect;

  StatePtr startState(new State(rsspace));
  StatePtr goalState(new State(rsspace));

  startvect = {3.0,2.0,0.0};
  goalvect = {2.0,1.5,M_PI/2.0};
  rsspace->copyFromReals(startState->getOMPLState(),startvect); 
  rsspace->copyFromReals(goalState->getOMPLState(),goalvect); 
  std::cout<<"ompl: "<<rsspace->distance(startState->getOMPLState(), goalState->getOMPLState())<<std::endl;
  double length;
  std::vector<rsmotion::algorithm::State> path;
  auto pair = testheuristic(startvect, goalvect);
  length = pair.first;
  path = pair.second;


  using namespace rsmotion::math;
  const float wheelbase = 0.44f;
  double turning_radius = 0.627;

  const Vec3f startPosition {startvect[1], 0.f, startvect[0]};
  const Quatf startOrientation { Vec3f{0,1,0}, Anglef::Radians(startvect[2]) };
  rsmotion::CarState carStart{{startPosition, startOrientation}, wheelbase};

  const Vec3f finishPosition {goalvect[1], 0.f, goalvect[0]};
  const Quatf finishOrientation { Vec3f{0,1,0}, Anglef::Radians(goalvect[2]) };
  const rsmotion::PointState finishPoint {finishPosition, finishOrientation};

  auto paths = EnumPaths(carStart, finishPoint, turning_radius);
  for (rsmotion::algorithm::Path path : paths){
      auto statePath = GetPath(carStart, path, mReader->resolution, TURNING_RADIUS);
      std::cout<<statePath.back().Phi()<<std::endl;
  }
  displayPath(obstacleLocation, path);
  //displayPaths(obstacleLocation, paths, carStart, 0.05, turning_radius, lat2real); 
  
  // Dubins
  DubinsPath dubins_path;
  std::vector<DubinsPath> dpaths;
  std::vector<rsmotion::algorithm::State> state_path;
  double start[] = {3.0,2.0,0.0+M_PI};
  double goal[] = {2.0,1.5,(M_PI*3.0/2.0)+M_PI};
  dubins_shortest_path(&dubins_path, start, goal, turning_radius); 

  dubins_path_sample_many(&dubins_path,  0.05, std::bind(&savePath, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, &state_path), NULL);

  dubins_paths(&dpaths, start, goal, turning_radius); 

  std::vector<std::vector<rsmotion::algorithm::State>> dstate_paths;
  for(DubinsPath dpath : dpaths){
      std::cout<<"length: "<<CONTXY2DISC(dubins_path_length(&dpath), mReader->resolution)<<std::endl;
      std::vector<rsmotion::algorithm::State> dstate_path;
      dubins_path_sample_many(&dpath,  0.05, std::bind(&savePath, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, &dstate_path), NULL);
      dstate_paths.push_back(dstate_path);
  }
  displayPath(obstacleLocation, state_path);
  std::cout<<"size: "<<dstate_paths.size()<<std::endl;
  displayStatePaths(obstacleLocation, dstate_paths, lat2real); 

  */ 
  return 0;
}
