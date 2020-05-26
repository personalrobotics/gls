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

// roscpp
#include "ros/ros.h"
// Custom header files
#include "gls/GLS.hpp"
#include "gls/event/ShortestPathEvent.hpp"
#include "gls/selector/ForwardSelector.hpp"

namespace po = boost::program_options;

/// Dummy collision checker that return true always.
/// This is bound to the stateValidityChecker of the ompl StateSpace.
/// \param[in] state The ompl state to check for validity.
bool isPointValid(const ompl::base::State* state) {
  double* values =
      state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  if (values[0] >= 0.4 && values[0] <= 0.6) {
    if (values[1] >= 0.4 && values[1] <= 0.6) {
      return false;
    }
  }
  return true;
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
  po::options_description desc("2D Map Planner Options");
  // clang-format off
  desc.add_options()
  ("help,h", "produce help message")
  ("graph,g", po::value<std::string>()->default_value("graph.graphml"), "graph specification");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }
  std::string graphLocation(vm["graph"].as<std::string>());
  graphLocation = "/home/adityavk/workspaces/lab-ws/src/gls/examples/graphs/" +
                  graphLocation;
  std::cout << graphLocation << std::endl;

  std::vector<float> source{0.1, 0.1};
  std::vector<float> target{0.9, 0.9};

  // Define the state space: R^2
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  std::function<bool(const ompl::base::State*)> isStateValid =
      std::bind(isPointValid, std::placeholders::_1);
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
    std::cout << "Best Path Cost: " << planner.getBestPathCost() << std::endl;
    return 0;
  }
  return -1;
}
