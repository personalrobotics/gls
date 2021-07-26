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

// For GIF
#include <Magick++.h>

// Custom header files
#include "gls/GLS.hpp"
#include <rsmotion/rsmotion.h>
#include "dubins.h"

using namespace gls::datastructures;
using namespace rsmotion::math;
typedef std::tuple<IVertex, VertexProperties, double, int> Neighbor;
typedef std::pair<double, double> xy_pt;
float WHEELBASE = 0.44;
float TURNING_RADIUS = 0.627;
int NUMTHETADIRS = 16;

void displayPoints(cv::Mat image, std::vector<xy_pt> points, double resolution, cv::Scalar color) {
  // Get state count

  // Obtain the image matrix
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  // TODO don't do this
  double scale = 300.0;
  int offsetx = -500;
  int offsety = 800;
  scale = 200.0;
  offsetx = -100;
  offsety = 0;
  for (xy_pt point : points){
    cv::Point uPoint((int)
            (DISCXY2CONT(point.first, resolution)*scale-offsetx), 
            (int)(numberOfRows - DISCXY2CONT(point.second, resolution)*scale)-offsety);

    cv::circle(image, uPoint, 1, color, cv::FILLED);
  }

}

// Make mushr car footprint for collision checking
std::vector<xy_pt> make_robot_footprint(double width, double length, double resolution){
    // Assuming car oriented at [0, 0, 0] and position is with respect to the rear axle
    std::vector<xy_pt> footprint;


    // Fits within frame better to floor for contact purposes
    double halfwidthd = std::floor(width/(2.0* resolution));
    double lengthd = std::floor(length/ resolution);
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

// Translate and rotate footprint around given position
// Assumes footprint and position are on lattice already 
std::vector<xy_pt> positionFootprint(std::vector<double> position, std::vector<xy_pt> footprint, double resolution){
    double angle = gls::io::DiscTheta2Cont((int)position[2], NUMTHETADIRS);
    double x, y, x_rot, y_rot;
    std::vector<xy_pt> rotated_footprint;

    for(xy_pt point : footprint){

        // Convert to real and Rotate
        x = DISCXY2CONT(point.first,resolution);
        y = DISCXY2CONT(point.second,resolution);
        x_rot = x*std::cos(angle) - y*std::sin(angle);
        y_rot = x*std::sin(angle) + y*std::cos(angle);
        point.first = CONTXY2DISC(x_rot,resolution);
        point.second = CONTXY2DISC(y_rot,resolution);

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
// Assuming points on lattice
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

  /*
   //debug
  std::string obstacleLocation("/home/schmittle/mushr/catkin_ws/src/gls/examples/blank.png");
  cv::Mat image = cv::imread(obstacleLocation, 1);
  displayPoints(image, rrobot_fp, 0.05, cv::Scalar(255,0,0));
  displayPoints(image, rblock_fp, 0.05, cv::Scalar(0,0,255));
  cv::imshow("Points", image);
  cv::waitKey(0);
  */

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
    vals.reserve(7);
    space->copyToReals(vals, state->getOMPLState());
    vals[0] = (double)CONTXY2DISC(vals[0], mReader->resolution);
    vals[1] = (double)CONTXY2DISC(vals[1], mReader->resolution);
    vals[2] = (double) gls::io::ContTheta2Disc(vals[2], NUMTHETADIRS);
    vals[4] = (double)CONTXY2DISC(vals[4], mReader->resolution);
    vals[5] = (double)CONTXY2DISC(vals[5], mReader->resolution);
    vals[6] = (double) gls::io::ContTheta2Disc(vals[6], NUMTHETADIRS);

    StatePtr newState(new State(space));
    space->copyFromReals(newState->getOMPLState(), vals);
    return newState;
}

// Reverse continuous state from lattice 
// This must be user-defined because it varies per state space
std::vector<double> lattice2real(std::vector<double> vals, gls::io::MotionPrimitiveReader *mReader){
    std::vector<double>real_vals;
    real_vals.reserve(7);
    real_vals[0] = (double)DISCXY2CONT(vals[0], mReader->resolution);
    real_vals[1] = (double)DISCXY2CONT(vals[1], mReader->resolution);
    real_vals[2] = (double) gls::io::DiscTheta2Cont(vals[2], NUMTHETADIRS);
    real_vals[3] = vals[3];
    real_vals[4] = (double)DISCXY2CONT(vals[4], mReader->resolution);
    real_vals[5] = (double)DISCXY2CONT(vals[5], mReader->resolution);
    real_vals[6] = (double) gls::io::DiscTheta2Cont(vals[6], NUMTHETADIRS);

    return real_vals;
}

// Given robot state, return block state assuming in contact
rsmotion::math::Vec3f getBlockPos(std::vector<double> car_pose){

    const Vec3f rBlockTransform = {0.0, 0.0, 0.05 + WHEELBASE};

    // Apply transform to car
    const Quatf carOrientation {Vec3f{0,1,0}, Anglef::Radians(car_pose[2])};
    Vec3f rotatedTransform = carOrientation.RotateVector(carOrientation, rBlockTransform);

    const Vec3f carPosition {car_pose[1], 0.0f, car_pose[0]};
    Vec3f blockPosition = carPosition + rotatedTransform;

    return blockPosition;
}

// Given block state and given side, return robot state assuming in contact
rsmotion::math::Vec3f getCarPos(std::vector<double> block_pose, int side){

    //using namespace rsmotion::math;
    const Vec3f blockTransform = {0.0, 0.0, -(0.05 + WHEELBASE)};

    // Apply transform to block
    Vec3f carPosition {block_pose[1], 0.f, block_pose[0]};

    const Quatf carOrientation { Vec3f{0,1,0}, Anglef::Radians(block_pose[2] + side*M_PI/2.0) };
    Vec3f rotatedTransform = carOrientation.RotateVector(carOrientation, blockTransform);
    carPosition += rotatedTransform;

    return carPosition;
}

// Save dubin's path to variable
int savePath(double q[3], double x, void* user_data, std::vector<rsmotion::algorithm::State>* saved_path) {
    rsmotion::algorithm::State dstate(q[0], q[1], q[2]-M_PI, false);
    saved_path->push_back(dstate);
    return 0;
}

// For each state in path finds if state is valid using isRobotValid
bool pathValid(std::vector<rsmotion::algorithm::State>  statePath, 
        std::vector<double> values,
        double resolution, 
        std::vector<xy_pt> robot_footprint, 
        std::vector<xy_pt> block_footprint,
        ompl::base::RealVectorBounds& bounds,
        std::function<std::vector<double>(std::vector<double>)> lat2real) {

    for (int j = 0; j < statePath.size(); ++j) {
        auto u = statePath[j];
        std::vector<double> uvec = {CONTXY2DISC(u.X(), resolution), 
                                    CONTXY2DISC(u.Y(), resolution), 
                                    gls::io::ContTheta2Disc(u.Phi(), NUMTHETADIRS), 
                                    values[3],
                                    values[4],
                                    values[5],
                                    values[6]};
        if(!isRobotValid(uvec, bounds, lat2real, robot_footprint, block_footprint, resolution)){
            return false;
        }
    }
    return true;
}

// For each state in path finds if state is valid using isRobotValid
bool pathValid(std::vector<rsmotion::algorithm::State>  statePath, 
        double* values,
        double resolution, 
        std::vector<xy_pt> robot_footprint, 
        std::vector<xy_pt> block_footprint,
        ompl::base::RealVectorBounds& bounds,
        std::function<std::vector<double>(std::vector<double>)> lat2real) {

    for (int j = 0; j < statePath.size(); ++j) {
        auto u = statePath[j];
        std::vector<double> uvec = {CONTXY2DISC(u.X(), resolution), 
                                    CONTXY2DISC(u.Y(), resolution), 
                                    gls::io::ContTheta2Disc(u.Phi(), NUMTHETADIRS), 
                                    values[3],
                                    values[4],
                                    values[5],
                                    values[6]};
        if(!isRobotValid(uvec, bounds, lat2real, robot_footprint, block_footprint, resolution)){
            return false;
        }
    }
    return true;
}

// Creates vector of target states given a end block position. 
// Assuming we do not care about contact point at goal
std::vector<StatePtr> createTargetStates(std::vector<double> goal_vec, std::shared_ptr<ompl::base::RealVectorStateSpace> space){
  std::vector<StatePtr> targetStates = {};
  std::cout<<"Goal: "<<goal_vec[0]<<" "<<goal_vec[1]<<" "<<goal_vec[2]<<std::endl;
  for(int i=0; i<4;i++){
    StatePtr targetState(new State(space));
    auto car_pos = getCarPos(goal_vec, i);

    std::cout<<"Side "<<i<<": "<<car_pos[2]<<" "<<car_pos[0]<<" "<<goal_vec[2]+i*M_PI/2.0<<std::endl;
    space->copyFromReals(targetState->getOMPLState(), 
            std::vector<double>{
            car_pos[2], 
            car_pos[0], 
            goal_vec[2]+i*M_PI/2.0, 
            i, 
            goal_vec[0], 
            goal_vec[1], 
            goal_vec[2]});
    targetStates.push_back(targetState);

  }
  return targetStates;
}

// Compute shortest dubins/reedshepp path for a given config
std::pair<std::vector<Neighbor>, std::vector<std::vector<rsmotion::algorithm::State>>>  computeReconfigures(
        std::vector<double> robot_state, // on lattice
        double resolution, 
        std::vector<xy_pt> robot_footprint, 
        std::vector<xy_pt> block_footprint, 
        ompl::base::RealVectorBounds& bounds, 
        std::shared_ptr<ompl::base::RealVectorStateSpace> space, 
        std::function<std::vector<double>(std::vector<double>)> lat2real) {

    std::vector<Neighbor> reconfigure_neighbors; 
    std::tuple<IVertex, VertexProperties, double, int> recon_neighbor;
    std::vector<std::vector<rsmotion::algorithm::State>> interpolated_paths;

    // Use reedshepp to find paths to faces
    // set the start position
    const Vec3f startPosition {DISCXY2CONT(robot_state[1], resolution), 0.0f, DISCXY2CONT(robot_state[0], resolution)};
    const Quatf startOrientation {Vec3f{0,1,0}, Anglef::Radians(gls::io::DiscTheta2Cont(robot_state[2], NUMTHETADIRS))};
    rsmotion::CarState carStart{{startPosition, startOrientation}, WHEELBASE};

    VertexProperties neighborProperties;
    IVertexHash hash;
    double length;
    int rs_index = 1;

    std::cout<<"contact type: "<<robot_state[3]<<std::endl;
    // Four sides of the block
    for (int i=0; i < 4; i++){
        if(robot_state[3] == i) // don't eval "switching" to the side we are already in contact with
            continue;

        // Target location
        double side_theta =  gls::io::DiscTheta2Cont(robot_state[6], NUMTHETADIRS) + i*M_PI/2.0;
        std::vector<double> block_pose = {DISCXY2CONT(robot_state[4], resolution),
                                        DISCXY2CONT(robot_state[5], resolution),
                                        gls::io::DiscTheta2Cont(robot_state[6], NUMTHETADIRS)};
        const Quatf finishOrientation { Vec3f{0,1,0}, Anglef::Radians(side_theta)};
        const Vec3f finishPosition = getCarPos(block_pose, i);
        const rsmotion::PointState finishPoint {finishPosition, finishOrientation};

        // All RS paths
        auto paths = EnumPaths(carStart, finishPoint, TURNING_RADIUS);

        // All Dubin's Paths
        std::vector<DubinsPath> dpaths;
        double start[] = {startPosition[2], startPosition[0], gls::io::DiscTheta2Cont(robot_state[2], NUMTHETADIRS)+M_PI};
        double goal[] = {finishPosition[2], finishPosition[0], side_theta+M_PI};
        
        double minLength = 1e5;
        int nx = CONTXY2DISC(finishPosition[2], resolution);
        int ny = CONTXY2DISC(finishPosition[0], resolution);
        int ntheta = gls::io::ContTheta2Disc(side_theta, NUMTHETADIRS);
        dubins_paths(&dpaths, start, goal, TURNING_RADIUS); 

        for (rsmotion::algorithm::Path path : paths){
            length = CONTXY2DISC(path.Length(TURNING_RADIUS), resolution);

            // Check for collisions with block
            if(length < minLength){
                auto rsStatePath = GetPath(carStart, path, resolution, TURNING_RADIUS);

                interpolated_paths.push_back(rsStatePath);

                bool valid = pathValid(rsStatePath, robot_state, 
                        resolution, robot_footprint, block_footprint, bounds, lat2real);

                // Choose shortest valid transition
                if(valid){
                    minLength = length;
                    neighborProperties = VertexProperties();

                    // Set state
                    StatePtr newState(new State(space));
                    space->copyFromReals(newState->getOMPLState(), std::vector<double>
                            {nx, ny, ntheta, i, robot_state[4], robot_state[5], robot_state[6]});
                    neighborProperties.setState(newState);

                    recon_neighbor = Neighbor(
                            hash(neighborProperties.getState()), neighborProperties, length, -rs_index);
                }
                rs_index++;
            }
        }
        if(minLength < 1e5){
            reconfigure_neighbors.push_back(recon_neighbor); // shortest valid reed shepp path
        }

        // Eval dubins
        minLength = 1e5;
        for(DubinsPath dpath : dpaths){
            length = CONTXY2DISC(dubins_path_length(&dpath), resolution);

            if(length < minLength){
                std::vector<rsmotion::algorithm::State> dstatePath;
                dubins_path_sample_many(&dpath, resolution, 
                        std::bind(&savePath, 
                            std::placeholders::_1, 
                            std::placeholders::_2, 
                            std::placeholders::_3, 
                            &dstatePath), NULL);

                interpolated_paths.push_back(dstatePath);

                // Check for collisions with block
                bool valid = pathValid(dstatePath, 
                      robot_state, resolution, robot_footprint, block_footprint, bounds, lat2real);
                // Choose shortest valid transition
                if(valid){
                    minLength = length;
                    neighborProperties = VertexProperties();

                    // Set state
                    StatePtr newState(new State(space));
                    space->copyFromReals(newState->getOMPLState(), std::vector<double>{nx, ny, ntheta, i, robot_state[4], robot_state[5], robot_state[6]});
                    neighborProperties.setState(newState);

                    recon_neighbor = Neighbor(hash(neighborProperties.getState()), neighborProperties, length, -(rs_index+100)); // 100 used to delineate dubins from rs for old interpolate
                }
                rs_index++;
            }
        }
        if(minLength < 1e5){
            reconfigure_neighbors.push_back(recon_neighbor); // shortest valid dubins path
        }
    }
    return std::pair<std::vector<Neighbor>, std::vector<std::vector<rsmotion::algorithm::State>>>{reconfigure_neighbors, interpolated_paths};
}

// Transition function from one state to another. Uses pre-computed reconfigures if in contact
std::vector<Neighbor> cached_transition_function(
        IVertex vi, 
        VertexProperties vp, 
        std::shared_ptr<ompl::base::RealVectorStateSpace> space, 
        gls::io::MotionPrimitiveReader *mReader,
        std::vector<xy_pt> robot_footprint, 
        std::vector<xy_pt> block_footprint,
        ompl::base::RealVectorBounds& bounds, 
        std::function<std::vector<double>(std::vector<double>)> lat2real,
        std::vector<Neighbor> recon_neighbors) {

        std::vector<Neighbor> neighbors; 

        // Get State
        double* values = vp.getState()->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

        // Neighbors
        int nx, ny, ntheta;
        IVertexHash hash;
        IVertex neighbor;
        double length;
        VertexProperties neighborProperties;
        double resolution = mReader->resolution;

        // If in contact use primitives
        if (values[3] >= 0){
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
                    length = CONTXY2DISC(mprim.length, resolution);

                    // Convert to real
                    std::vector<double> car_pose = {DISCXY2CONT(nx, resolution),
                                                    DISCXY2CONT(ny, resolution),
                                                    gls::io::DiscTheta2Cont(ntheta, NUMTHETADIRS)};
                    const Vec3f blockPosition =  getBlockPos(car_pose);

                    // Set state
                    // TODO (schmittle) this seems inefficient to make a new state
                    neighborProperties = VertexProperties();
                    StatePtr newState(new State(space));
                    space->copyFromReals(newState->getOMPLState(), 
                            std::vector<double>{nx, ny, ntheta, values[3], 
                            CONTXY2DISC(blockPosition[2], resolution), 
                            CONTXY2DISC(blockPosition[0], resolution), 
                            values[6]+(ntheta-values[2])});
                    neighborProperties.setState(newState);

                    neighbors.push_back(Neighbor
                            (hash(neighborProperties.getState()), 
                             neighborProperties, 
                             length, 
                             mprim.motprimID));
                }
            }

            // Compute reconfigures
            for(Neighbor recon_neighbor : recon_neighbors){
                // Set state
                std::vector<double> state;
                StatePtr newState(new State(space));
                neighborProperties = VertexProperties();
                space->copyToReals(state, std::get<1>(recon_neighbor).getState()->getOMPLState());

                //std::cout<<"curr state: "<<values[0]<<", "<<values[1]<<", "<<values[2]<<std::endl;
                //std::cout<<"recon state: "<<state[0]<<", "<<state[1]<<", "<<state[2]<<std::endl;
                double cont_x = DISCXY2CONT(state[0], resolution);
                double cont_y = DISCXY2CONT(state[1], resolution);
                double cont_angle = -1 *gls::io::DiscTheta2Cont(NUMTHETADIRS - values[2], NUMTHETADIRS);
                //std::cout<<"change: "<<CONTXY2DISC(cont_x*cos(cont_angle) - cont_y*sin(cont_angle), resolution)<<", "<<CONTXY2DISC(cont_x*sin(cont_angle) + cont_y*cos(cont_angle), resolution) <<std::endl;
                // translate/rotate
                state[0] = CONTXY2DISC(cont_x*cos(cont_angle) - cont_y*sin(cont_angle), resolution) + values[0];
                state[1] = CONTXY2DISC(cont_x*sin(cont_angle) + cont_y*cos(cont_angle), resolution) + values[1];
                state[2] = ((int)(state[2] + values[2])) % NUMTHETADIRS;
                state[3] = (int)(values[3] + state[3])%4;
                state[4] = values[4];
                state[5] = values[5];
                state[6] = values[6];

                space->copyFromReals(newState->getOMPLState(), state);
                neighborProperties.setState(newState);

                recon_neighbor = Neighbor
                    (hash(newState), neighborProperties, std::get<2>(recon_neighbor), std::get<3>(recon_neighbor));

                neighbors.push_back(recon_neighbor); // shortest valid reedshepp/dubins path
            }
        }
        // Not in contact
        else{
            /*
            std::vector<Neighbor> reconfigure_neighbors = 
              computeReconfigures(std::vector<double>{values[0], values[1], values[2], values[3], values[4], values[5], values[6]}, resolution, 
                robot_footprint, block_footprint, bounds, space, lat2real).first;
                */
            std::cout<<"not in contact"<<std::endl;
            std::pair<std::vector<Neighbor>, std::vector<std::vector<rsmotion::algorithm::State>>> recon_neighbors = 
                computeReconfigures(std::vector<double>{values[0], values[1], values[2], values[3], values[4], values[5], values[6]}, resolution, 
                robot_footprint, block_footprint, bounds, space, lat2real);
            std::cout<<"preinfo: "<<recon_neighbors.second.size()<<std::endl;
            std::cout<<values[0]<<" "<<values[1]<<" "<<values[2]<<" "<<values[3]<<" "<<values[4]<<" "<<values[5]<<" "<<values[6]<<std::endl;
            auto reconfigure_neighbors = recon_neighbors.first;
            std::vector<double> test_state;
            space->copyToReals(test_state, std::get<1>(recon_neighbors.first[1]).getState()->getOMPLState());
            std::cout<<std::get<3>(recon_neighbors.first[1])<<" "<<test_state[0]<<" "<<test_state[1]<<" "<<test_state[2]<<std::endl;

            for(Neighbor recon_neighbor : reconfigure_neighbors){
                neighbors.push_back(recon_neighbor); // shortest valid reedshepp/dubins path
            }
        }
    return neighbors;
}

// Transition function from one state to another. Computes reconfigures each time
std::vector<Neighbor> transition_function(
        IVertex vi, 
        VertexProperties vp, 
        std::shared_ptr<ompl::base::RealVectorStateSpace> space, 
        gls::io::MotionPrimitiveReader *mReader,
        std::vector<xy_pt> robot_footprint, 
        std::vector<xy_pt> block_footprint,
        ompl::base::RealVectorBounds& bounds, 
        std::function<std::vector<double>(std::vector<double>)> lat2real) {

    std::vector<Neighbor> neighbors; 
    Neighbor recon_neighbor;

    // Get State
    double* values = vp.getState()->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    // Neighbors
    int nx, ny, ntheta;
    IVertexHash hash;
    IVertex neighbor;
    double length;
    VertexProperties neighborProperties;

    // If in contact use primitives
    if (values[3] >= 0){
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

                // Convert to real
                std::vector<double> car_pose = {DISCXY2CONT(nx, mReader->resolution),
                                                DISCXY2CONT(ny, mReader->resolution),
                                                gls::io::DiscTheta2Cont(ntheta, NUMTHETADIRS)};
                const Vec3f blockPosition =  getBlockPos(car_pose);

                // Set state
                // TODO (schmittle) this seems inefficient to make a new state
                neighborProperties = VertexProperties();
                StatePtr newState(new State(space));
                space->copyFromReals(newState->getOMPLState(), 
                        std::vector<double>{nx, ny, ntheta, values[3], 
                        CONTXY2DISC(blockPosition[2], mReader->resolution), 
                        CONTXY2DISC(blockPosition[0], mReader->resolution), 
                        values[6]+(ntheta-values[2])});
                neighborProperties.setState(newState);

                neighbors.push_back(Neighbor
                        (hash(neighborProperties.getState()), 
                         neighborProperties, 
                         length, 
                         mprim.motprimID));
            }
        }
    }
    std::vector<Neighbor> reconfigure_neighbors = 
      computeReconfigures(std::vector<double>{values[0], values[1], values[2], values[3], values[4], values[5], values[6]}, mReader->resolution, 
        robot_footprint, block_footprint, bounds, space, lat2real).first;

    for(Neighbor recon_neighbor : reconfigure_neighbors){
        neighbors.push_back(recon_neighbor); // shortest valid reedshepp/dubins path
    }
    
    return neighbors;
}

void positionPath(std::vector<rsmotion::algorithm::State>& path, std::vector<double> start_pose){
    //std::cout<<start_pose[0]<<" "<<start_pose[1]<<" "<<start_pose[2]<<std::endl;
    for(int i=0; i < path.size(); i++){
        rsmotion::algorithm::State state = path[i];
        double cont_angle = -(2*M_PI-start_pose[2]);

        // translate/rotate
        double x = state.X()*cos(cont_angle) - state.Y()*sin(cont_angle) + start_pose[0];
        double y = state.X()*sin(cont_angle) + state.Y()*cos(cont_angle) + start_pose[1];
        double phi = state.Phi() + start_pose[2];
        //std::cout<<"    "<<state.X()<<" "<<state.Y()<<" "<<state.Phi()<<" | "<<x<<" "<<y<<" "<<phi<<std::endl;
        path[i] = rsmotion::algorithm::State{x, y, phi};
    }
}

std::vector<StatePtr> cached_interpolate(StatePtr startState, StatePtr endState, 
        int motPrimID, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader, 
        std::vector<std::vector<rsmotion::algorithm::State>> reconfigure_paths, 
        std::function<std::pair<std::vector<Neighbor>, std::vector<std::vector<rsmotion::algorithm::State>>>(std::vector<double>)> getTransition) {

    double* startValues = 
        startState->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    double* endValues = 
        endState->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::vector<StatePtr> states;
    double res = mReader->resolution;
    std::cout<<"start: "<<motPrimID<<" "<<startValues[0]<<" "<<startValues[1]<<" "<<startValues[2]<<" "<<startValues[3]<<" "<<startValues[4]<<" "<<startValues[5]<<" "<<startValues[6]<<std::endl;
    std::cout<<"   end: "<<endValues[0]<<" "<<endValues[1]<<" "<<endValues[2]<<" "<<endValues[3]<<" "<<endValues[4]<<" "<<endValues[5]<<" "<<endValues[6]<<std::endl;

    std::vector<gls::io::MotionPrimitive> mprims = mReader->mprimV;
    if (motPrimID >= 0){ // a true motion primitive
        for(gls::io::MotionPrimitive mprim : mprims){
            if(mprim.motprimID == motPrimID && mprim.starttheta_c == startValues[2]){
                for(gls::io::pt_xyt point : mprim.intermptV){
                    StatePtr newState(new State(space));

                    // Convert to real
                    std::vector<double> car_pose = {(double)point.x + DISCXY2CONT(startValues[0], res),
                                                    (double)point.y + DISCXY2CONT(startValues[1], res),
                                                    (double)point.theta};
                    const rsmotion::math::Vec3f blockPosition =  getBlockPos(car_pose);

                    space->copyFromReals(newState->getOMPLState(), 
                            std::vector<double>{car_pose[0], 
                                                car_pose[1], 
                                                car_pose[2], 
                                                startValues[3], 
                                                blockPosition[2],
                                                blockPosition[0],
                                                car_pose[2]});
                    states.push_back(newState);
                }
            }
        }
    }
    else{ // a reed shepp or dubins path
        std::vector<rsmotion::algorithm::State> path;
        std::vector<double> contStart = {DISCXY2CONT(startValues[0], res), DISCXY2CONT(startValues[1], res), gls::io::DiscTheta2Cont(startValues[2], NUMTHETADIRS)};
        std::cout<<startValues[3]<<std::endl;

        // Compute Approach 
        if(startValues[3] == -1){
            std::vector<std::vector<rsmotion::algorithm::State>> approach_neighbors = 
              getTransition(std::vector<double>{startValues[0], startValues[1], startValues[2], startValues[3], startValues[4], startValues[5], startValues[6]}).second;
            
            std::cout<<"info: "<<motPrimID<< " "<<approach_neighbors.size()<<std::endl;
            std::cout<<startValues[0]<<" "<<startValues[1]<<" "<<startValues[2]<<" "<<startValues[3]<<" "<<startValues[4]<<" "<<startValues[5]<<" "<<startValues[6]<<std::endl;
            if(abs(motPrimID) >= 100){
                path = approach_neighbors[abs(motPrimID)-101];
            }
            else{
                path = approach_neighbors[abs(motPrimID)-1];
            }
        }
        else{ // Use cached reconfigures
            if(abs(motPrimID) >= 100){
                path = reconfigure_paths[abs(motPrimID)-101];
                positionPath(path, contStart);  
            }
            else{
                path = reconfigure_paths[abs(motPrimID)-1];
                positionPath(path, contStart);  
            }

        }
        for (int i = 0; i < path.size(); ++i) {
            auto u = path[i];
            std::vector<double> uvec = {u.X(), u.Y(), u.Phi()};
            if(startValues[3] == -1){
                std::cout<<uvec[0]<<", "<<uvec[1]<<", "<<uvec[2]<<std::endl;
            }
            StatePtr newState(new State(space));
            space->copyFromReals(newState->getOMPLState(), 
                    std::vector<double>{uvec[0], 
                                        uvec[1], 
                                        uvec[2], 
                                        -1, 
                                        DISCXY2CONT(startValues[4], res),
                                        DISCXY2CONT(startValues[5], res),
                                        gls::io::DiscTheta2Cont(startValues[6], NUMTHETADIRS)});
            states.push_back(newState);
        }
    }
    return states;
}

std::vector<StatePtr> interpolate(StatePtr startState, StatePtr endState, int motPrimID, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){

    double* startValues = 
        startState->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    double* endValues = 
        endState->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    std::vector<StatePtr> states;
    double res = mReader->resolution;

    std::vector<gls::io::MotionPrimitive> mprims = mReader->mprimV;
    if (motPrimID >= 0){ // a true motion primitive
        for(gls::io::MotionPrimitive mprim : mprims){
            if(mprim.motprimID == motPrimID && mprim.starttheta_c == startValues[2]){
                for(gls::io::pt_xyt point : mprim.intermptV){
                    StatePtr newState(new State(space));

                    // Convert to real
                    std::vector<double> car_pose = {(double)point.x + DISCXY2CONT(startValues[0], res),
                                                    (double)point.y + DISCXY2CONT(startValues[1], res),
                                                    (double)point.theta};
                    const rsmotion::math::Vec3f blockPosition =  getBlockPos(car_pose);

                    space->copyFromReals(newState->getOMPLState(), 
                            std::vector<double>{car_pose[0], 
                                                car_pose[1], 
                                                car_pose[2], 
                                                startValues[3], 
                                                blockPosition[2],
                                                blockPosition[0],
                                                car_pose[2]});
                    states.push_back(newState);
                }
            }
        }
    }
    else{ // a reed shepp or dubins path
        
        // re-formatting
        const Vec3f startPosition {DISCXY2CONT(startValues[1], res)
                                , 0.0f, DISCXY2CONT(startValues[0], res)};
        const Quatf startOrientation { Vec3f{0,1,0}, Anglef::Radians(
                gls::io::DiscTheta2Cont(startValues[2], NUMTHETADIRS)) };
        rsmotion::CarState carStart{{startPosition, startOrientation}, WHEELBASE};

        const Vec3f  finishPosition = {DISCXY2CONT(endValues[1], res),
                                        0.0f, DISCXY2CONT(endValues[0], res)};
        const Quatf finishOrientation { Vec3f{0,1,0}, Anglef::Radians(
                gls::io::DiscTheta2Cont(endValues[2], NUMTHETADIRS)) };
        const rsmotion::PointState finishPoint {finishPosition, finishOrientation};
        std::vector<rsmotion::algorithm::State> path;
        
        if(abs(motPrimID) >= 100){
            // dubins
            std::vector<DubinsPath> dpaths;
            double start[] = {startPosition[2], startPosition[0], gls::io::DiscTheta2Cont(startValues[2], NUMTHETADIRS)+M_PI};
            double goal[] = {finishPosition[2], finishPosition[0], gls::io::DiscTheta2Cont(endValues[2], NUMTHETADIRS)+M_PI};
            std::cout<<finishPosition[2]<<" "<< finishPosition[0]<<" "<< endValues[2]<<std::endl;
            dubins_paths(&dpaths, start, goal, TURNING_RADIUS); 
            dubins_path_sample_many(&dpaths[abs(motPrimID)-100],  res, 
                    std::bind(&savePath, 
                        std::placeholders::_1, 
                        std::placeholders::_2, 
                        std::placeholders::_3, 
                        &path), NULL);
       }
        else{
            // Reed Shepp

            // Get the path 
            auto paths = EnumPaths(carStart, finishPoint, TURNING_RADIUS);
            path = rsmotion::GetPath(carStart, paths[abs(motPrimID)-1], res, TURNING_RADIUS);
        }

        for (int i = 0; i < path.size(); ++i) {
            auto u = path[i];
            std::vector<double> uvec = {u.X(), u.Y(), u.Phi()};
            StatePtr newState(new State(space));
            space->copyFromReals(newState->getOMPLState(), 
                    std::vector<double>{uvec[0], 
                                        uvec[1], 
                                        uvec[2], 
                                        -1, 
                                        DISCXY2CONT(startValues[4], res),
                                        DISCXY2CONT(startValues[5], res),
                                        gls::io::DiscTheta2Cont(startValues[6], NUMTHETADIRS)});
            states.push_back(newState);
        }
    }
    return states;
}

// Euclidean for now
double reconfigure_heuristic(StatePtr v, StatePtr target, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){
    double* values = 
        v->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* goal_values = 
        target->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    std::vector<double> cont_values = {DISCXY2CONT(values[0], mReader->resolution), DISCXY2CONT(values[1], mReader->resolution), 
        DISCXY2CONT(values[4], mReader->resolution), 
        DISCXY2CONT(values[5], mReader->resolution)};

    std::vector<double> cont_goal_values = {DISCXY2CONT(goal_values[4], mReader->resolution), DISCXY2CONT(goal_values[5], mReader->resolution)};

    // Approach 
    //double approach_heuristic = abs(sqrt(pow((cont_values[2] - cont_values[0]),2) + pow((cont_values[3] - cont_values[1]),2))-0.5); // 0.5 compensating for wheelbase + block halfwidth

    // Pushing 
    double pushing_heuristic = sqrt(pow((cont_goal_values[0] - cont_values[2]),2) + pow((cont_goal_values[1] - cont_values[3]),2));

    //return CONTXY2DISC(approach_heuristic + pushing_heuristic, mReader->resolution);
    return CONTXY2DISC(pushing_heuristic, mReader->resolution);
}

double rsheuristic(StatePtr v, StatePtr target, std::shared_ptr<ompl::base::RealVectorStateSpace> space, gls::io::MotionPrimitiveReader *mReader){

    // Approach 
    double* values = 
        v->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* goal_values = 
        target->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;


    // set the wheelbase to 0.44 meter

    // set the start position
    const Vec3f startPosition {DISCXY2CONT(values[1], mReader->resolution)
                            , 0.f, DISCXY2CONT(values[0], mReader->resolution)};

    // set the orientation
    const Quatf startOrientation { Vec3f{0,1,0}, Anglef::Radians(
            gls::io::DiscTheta2Cont(values[2], NUMTHETADIRS)) };

    // create the initial CarState
    rsmotion::CarState carStart{{startPosition, startOrientation}, WHEELBASE};

    const Vec3f finishPosition {DISCXY2CONT(goal_values[1], mReader->resolution), 0.f, 
        DISCXY2CONT(goal_values[0], mReader->resolution)};
    const Quatf finishOrientation { Vec3f{0,1,0}, Anglef::Radians(
            gls::io::DiscTheta2Cont(goal_values[2], NUMTHETADIRS)) };
    const rsmotion::PointState finishPoint {finishPosition, finishOrientation};

    const auto path = SearchShortestPath(carStart, finishPoint, TURNING_RADIUS);

    // Pushing
    DubinsPath dubins_path;
    double start[] = {startPosition[0], startPosition[2], startPosition[1] + M_PI};
    double goal[] = {finishPosition[0], finishPosition[2], finishPosition[1] + M_PI};
    dubins_shortest_path(&dubins_path, start, goal, TURNING_RADIUS); 
    // TODO full heuristic with block

    return CONTXY2DISC(path.Length(TURNING_RADIUS), mReader->resolution);
}

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
void displayPath(std::string obstacleFile, std::shared_ptr<ompl::geometric::PathGeometric> path, double resolution) {
  // Get state count
  int pathSize = path->getStateCount();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  // TODO don't do this
  double scale = 300.0;
  int offsetx = 300;
  int offsety = -100;
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
                && gls::io::ContTheta2Disc(v[2], NUMTHETADIRS) == gls::io::ContTheta2Disc(goalVals[2], NUMTHETADIRS)){
            cutshort = true;
            u = goalVals;
        }
    }

    std::vector<double> uvec = {u[0], u[1], u[2]};
    std::vector<double> block_vec = {u[4], u[5], u[6]};

    if(i%car_int == 0 || i==pathSize-1 || cutshort){
        if(i == 0){
            box_color = cv::Scalar(0, 255, 0);
        }
        if(i == pathSize-1 || cutshort){
            box_color = cv::Scalar(0, 0, 255);
            std::cout<<u[4]<<" "<<u[5]<<" "<<u[6]<<std::endl;
            std::cout<<u[0]<<" "<<u[1]<<" "<<u[2]<<std::endl;
            cv::Point bPoint((int)(u[4]*scale-offsetx), (int)(numberOfRows - u[5]*scale)-offsety);
            cv::circle(image, bPoint, 1, cv::Scalar(255, 0, 0), cv::FILLED);
        }

        // Car
        makeBox(image, box_color, uvec, {0.44, 0.27}, false, scale, offsetx, offsety, numberOfRows);

        // Block
        makeBox(image, box_color, block_vec, {0.1, 0.1}, true, scale, offsetx, offsety, numberOfRows);

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

/// Saves gif of path
void saveGif(std::string name, std::string obstacleFile, std::shared_ptr<ompl::geometric::PathGeometric> path, double resolution) {
  // Get state count
  int pathSize = path->getStateCount();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  std::list<Magick::Image> images;
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  // TODO don't do this
  double scale = 300.0;
  int offsetx = 300;
  int offsety = -100;
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
                && gls::io::ContTheta2Disc(v[2], NUMTHETADIRS) == gls::io::ContTheta2Disc(goalVals[2], NUMTHETADIRS)){
            cutshort = true;
            u = goalVals;
        }
    }

    std::vector<double> uvec = {u[0], u[1], u[2]};
    std::vector<double> block_vec = {u[4], u[5], u[6]};


    if(i == 0){
        box_color = cv::Scalar(0, 255, 0);
    }
    if(i == pathSize-1 || cutshort){
        box_color = cv::Scalar(0, 0, 255);
        std::cout<<u[4]<<" "<<u[5]<<" "<<u[6]<<std::endl;
        std::cout<<u[0]<<" "<<u[1]<<" "<<u[2]<<std::endl;
        cv::Point bPoint((int)(u[4]*scale-offsetx), (int)(numberOfRows - u[5]*scale)-offsety);
        cv::circle(image, bPoint, 1, cv::Scalar(255, 0, 0), cv::FILLED);
    }

    // Car
    makeBox(image, box_color, uvec, {0.44, 0.27}, false, scale, offsetx, offsety, numberOfRows);

    std::cout<<block_vec[0]<<" "<<block_vec[1]<<" "<<block_vec[2]<<std::endl;
    // Block
    makeBox(image, box_color, block_vec, {0.1, 0.1}, true, scale, offsetx, offsety, numberOfRows);

    box_color = cv::Scalar(255, 0, 0);

    cv::circle(image, uPoint, 1, cv::Scalar(255, 0, 0), cv::FILLED);
    Magick::Image mgk(image.cols, image.rows, "BGR", Magick::CharPixel, (char *)image.data);
    images.push_back(mgk);
    image = cv::imread(obstacleFile, 1); // Probs better way todo this
    if(cutshort){
        break;
    }
  }
  Magick::writeImages(images.begin(), images.end(), name);
}

int main (int argc, char const *argv[]) {

  // Load Motion Primitives
  gls::io::MotionPrimitiveReader* mReader = new gls::io::MotionPrimitiveReader();
  //TODO (schmittle) don't hardcode paths
  //mReader->ReadMotionPrimitives("/home/schmittle/mushr/catkin_ws/src/gls/examples/mushr.mprim",
   //      "/home/schmittle/mushr/catkin_ws/src/gls/examples/mushr.json");
  mReader->ReadMotionPrimitives("/home/schmittle/mushr/catkin_ws/src/gls/examples/pushr_sandpaper.mprim",
          "/home/schmittle/mushr/catkin_ws/src/gls/examples/pushr_sandpaper.json");
  std::string obstacleLocation("/home/schmittle/mushr/catkin_ws/src/gls/examples/blank.png");

  // Define the state space: R^4
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(7);
  auto bounds = ompl::base::RealVectorBounds(7); 
  bounds.setLow(-100.0); // TODO set real bounds, although I don't think these are used
  bounds.setHigh(100.0);
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();
  
  // For collision checker
  std::vector<xy_pt> robot_footprint = make_robot_footprint(0.27, 0.44, mReader->resolution);
  std::vector<xy_pt> block_footprint = make_block_footprint(0.1, mReader->resolution);

  // Space Information
  std::function<std::vector<double>(std::vector<double>)> lat2real = std::bind(&lattice2real, std::placeholders::_1, mReader);
  // Just check that it is "within bounds"
  std::function<bool(const ompl::base::State*)> isStateValid
      = std::bind(&isStatePointValid, std::placeholders::_1, bounds, lat2real);
  // Actually check for collision with obstacles
  //std::function<bool(const ompl::base::State*)> isStateValid
      //= std::bind(&isRobotStateValid, std::placeholders::_1, bounds, lat2real, robot_footprint, block_footprint, mReader->resolution);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // In real space 
  std::vector<double> start_vec = {3, 2, 0, -1, 2, 2, 0};
  StatePtr sourceState(new State(space));
  // car_x, car_y, car_theta, contact type, block_x, block_y, block_theta
  space->copyFromReals(sourceState->getOMPLState(), start_vec);

  // block_x, block_y, block_theta
  //std::vector<StatePtr> targetStates = createTargetStates({1.75, 1.95, 2}, space);
  std::vector<StatePtr> targetStates = createTargetStates({1.75, 1.95, 2}, space);

  //StatePtr targetState(new State(space));
  //space->copyFromReals(targetState->getOMPLState(), std::vector<double>{1, 2, M_PI, 1, 2, 2, 0});
  //TODO specify multiple goals

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(sourceState->getOMPLState());
  //pdef->setGoalState(targetState->getOMPLState());

  // Setup planner
  gls::GLS planner(si);

  // Pre-compute reconfigures
  std::vector<double> car_pose = {0.0,0.0,0.0};
  const rsmotion::math::Vec3f blockPosition =  getBlockPos(car_pose);
  std::pair<std::vector<Neighbor>, std::vector<std::vector<rsmotion::algorithm::State>>> rNeighbors = 
      computeReconfigures(
        std::vector<double>{0,0,0,0,
            CONTXY2DISC(blockPosition[2], mReader->resolution), 
            CONTXY2DISC(blockPosition[0], mReader->resolution),0}, 
        mReader->resolution, 
        robot_footprint, block_footprint, bounds, space, lat2real);

  std::vector<Neighbor> reconfigure_neighbors =  rNeighbors.first;

  // For interpolate
  std::function<std::pair<std::vector<Neighbor>, std::vector<std::vector<rsmotion::algorithm::State>>>
      (std::vector<double>)>simplified_transition 
      = std::bind(&computeReconfigures, std::placeholders::_1, 
              mReader->resolution, robot_footprint, block_footprint, bounds, space, lat2real);

  planner.setImplicit(
          std::bind(&fit_state2lattice,  
                std::placeholders::_1, 
                mReader,
                space),
          std::bind(&cached_transition_function, 
                std::placeholders::_1, 
                std::placeholders::_2, 
                space, 
                mReader,
                robot_footprint,
                block_footprint,
                bounds,
                lat2real,
                reconfigure_neighbors),
          std::bind(&cached_interpolate, 
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3,
              space, 
              mReader,
              rNeighbors.second,
              simplified_transition)
          /*
          std::bind(&transition_function, 
                std::placeholders::_1, 
                std::placeholders::_2, 
                space, 
                mReader,
                robot_footprint,
                block_footprint,
                bounds,
                lat2real),
          std::bind(&interpolate, 
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3,
              space, 
              mReader)
                */
          );
  //planner.setHeuristic(std::bind(&rsheuristic,  
  planner.setHeuristic(std::bind(&reconfigure_heuristic,  
                std::placeholders::_1, 
                std::placeholders::_2, 
                space, mReader));

  auto event = std::make_shared<gls::event::ShortestPathEvent>();
  auto selector = std::make_shared<gls::selector::ForwardSelector>();
  planner.setEvent(event);
  planner.setSelector(selector);

  planner.setup();
  planner.setMultipleGoals(targetStates);
  planner.setProblemDefinition(pdef);

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
    displayPath(obstacleLocation, path, mReader->resolution); 
    saveGif("test.gif", obstacleLocation, path, mReader->resolution); 
    planner.clear();
  }

  /* debug stuff */

  return 0;
}
