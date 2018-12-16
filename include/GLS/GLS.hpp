/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_GLS_HPP_
#define GLS_GLS_HPP_

// STL headers
#include <vector>
#include <string> 
#include <unordered_set>
#include <queue>
#include <exception>

// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include "GLS/Event.hpp"
#include "GLS/Selector.hpp"
#include "GLS/Datastructures/Graph.hpp"

namespace GLS {

/// The OMPL Planner class that implements the algorithm
class GLS: public ompl::base::Planner
{
public:
  /// Constructor
  /// \param[in] si The OMPL space information manager
  explicit GLS(const ompl::base::SpaceInformationPtr &si);

  /// Destructor
  ~GLS(void);

  /// Set the problem definition and define the start, goal.
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);

  /// Solve the planning problem.
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

  /// Setup the planner.
  void setup() override;

  /// Clear the planner setup.
  void clear() override;

  // Setters and const Getters.
  void setEvent(gls::event::Event event);
  // gls::event::ConstEventPtr getEvent() const;

  void setSelector(gls::selector::Selector selector);
  // gls::selector::ConstSelectorPtr getSelector() const;

private:
  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// Event
  gls::event::Event mEvent;

  /// Selector
  gls::selector::Selector mSelector;

  /// The fixed roadmap over which the search is done.
  Graph mGraph;

  /// Source vertex.
  Vertex mStartVertex;

  /// Goal vertex.
  Vertex mGoalVertex;

  // Search Methods
  void extendSearchTree();
  void rewireSearchTree();
  void evaluateSearchTree();
}

#endif // GLS_GLS_HPP_