// Standard C++ libraries
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>

// Boost libraries
#include <boost/function.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/program_options.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/shared_ptr.hpp>

// OMPL base libraries
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// roscpp
#include "ros/ros.h"

// Custom header files
#include "gls/GLS.hpp"
#include "gls/event/ShortestPathEvent.hpp"
#include "gls/selector/ForwardSelector.hpp"

namespace po = boost::program_options;

/// Check if the point is within defined hyperrectangle
/// This is bound to the stateValidityChecker of the ompl StateSpace
/// \param[in] image Obstacle image (grayscale)
/// \param[in] state The ompl state to check for validity
/// \return True if the state is collision-free
bool isPointValid(cv::Mat image, const ompl::base::State* state) {
  // Obtain the state values
  double* values =
      state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  // Get the required point on the map
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;
  double x_point = values[0] * numberOfColumns;
  double y_point = (1 - values[1]) * numberOfRows;
  cv::Point point((int)x_point, (int)y_point);

  // Collision Check
  int intensity = (int)image.at<uchar>(point.y, point.x);
  if (intensity == 0) {
    return false;
  }
  return true;
}

/// Displays path
/// \param[in] obstacleFile The file with obstacles stored
/// \param[in] path OMPL path
void displayPath(std::string obstacleFile,
                 std::shared_ptr<ompl::geometric::PathGeometric> path) {
  // Get state count
  int pathSize = path->getStateCount();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  for (int i = 0; i < pathSize - 1; ++i) {
    auto uState = path->getState(i);
    auto vState = path->getState(i + 1);
    double* u =
        uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v =
        vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    cv::Point uPoint((int)(u[0] * numberOfColumns),
                     (int)((1 - u[1]) * numberOfRows));
    cv::Point vPoint((int)(v[0] * numberOfColumns),
                     (int)((1 - v[1]) * numberOfRows));

    cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 3);
  }

  cv::imshow("Solution Path", image);
  cv::waitKey(0);
}

/// Creates an OMPL state from state values.
/// \param[in] space The ompl space the robot is operating in.
/// \param[in] x The x-coordinate.
/// \param[in] y The y-coorindate.
ompl::base::ScopedState<ompl::base::RealVectorStateSpace> make_state(
    const ompl::base::StateSpacePtr space, double x, double y) {
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state(space);
  double* values =
      state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  values[0] = x;
  values[1] = y;
  return state;
}

/// The main function.
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test2d_image");

  std::vector<float> source{0.1, 0.1};
  std::vector<float> target{0.9, 0.9};
  std::string graphLocation =
      "/home/adityavk/workspaces/lab-ws/src/gls/examples/graph_400.graphml";
  std::string obstacleLocation =
      "/home/adityavk/workspaces/lab-ws/src/gls/examples/world.png";

  // Define the state space: R^2
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  cv::Mat image = cv::imread(obstacleLocation, 0);
  std::function<bool(const ompl::base::State*)> isStateValid =
      std::bind(isPointValid, image, std::placeholders::_1);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(make_state(space, source[0], source[1]));
  pdef->setGoalState(make_state(space, target[0], target[1]));

  // Setup planner
  gls::GLS planner(si);
  planner.setConnectionRadius(0.04);
  planner.setCollisionCheckResolution(0.1);
  planner.setRoadmap(graphLocation);

  auto event = std::make_shared<gls::ShortestPathEvent>();
  auto selector = std::make_shared<gls::ForwardSelector>();
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
    auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
        pdef->getSolutionPath());
    std::cout << "Solution Path Cost: " << planner.getBestPathCost()
              << std::endl;
    std::cout << "Number of Edge Evaluations: "
              << planner.getNumberOfEdgeEvaluations() << std::endl;
    displayPath(obstacleLocation, path);
    return 0;
  }

  return -1;
}
