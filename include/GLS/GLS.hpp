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

// GLS headers
#include "GLS/Event/Event.hpp"
#include "GLS/Selector/Selector.hpp"
#include "GLS/Datastructures/Graph.hpp"

namespace gls {

/// The OMPL Planner class that implements the algorithm.
class GLS: public ompl::base::Planner
{
public:
  /// Constructor.
  /// \param[in] si The OMPL space information manager.
  explicit GLS(const ompl::base::SpaceInformationPtr &si);

  /// Destructor.
  ~GLS(void);

  /// Set the problem definition and define the start, goal.
  /// \param[in] pdef OMPL Problem Definition.
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);

  /// Solve the planning problem.
  /// \param[in] ptc OMPL Planning Termination Condition.
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

  /// Setup the planner.
  void setup() override;

  /// Clear the planner setup.
  void clear() override;

  /// Set the event to be used by GLS.
  /// \param[in] event Event that defines the trigger condition.
  void setEvent(gls::event::EventPtr event);

  /// Returns the event used by the algorithm.
  gls::event::ConstEventPtr getEvent() const;

  /// Set the selector to be used by GLS.
  /// \param[in] selector Selector that defines the evaluation strategy.
  void setSelector(gls::selector::SelectorPtr selector);

  /// Returns the selector used by the algorithm.
  gls::selector::ConstSelectorPtr getSelector() const;

private:
  /// Extends the search tree forwards.
  void extendSearchTree();

  /// Rewires the search tree when edge costs change.
  void rewireSearchTree();

  /// Evaluates the search tree when the extension pauses.
  void evaluateSearchTree();

  /// The pointer to the OMPL state space.
  ompl::base::StateSpacePtr mSpace;

  /// Event
  gls::event::EventPtr mEvent;

  /// Selector
  gls::selector::SelectorPtr mSelector;

  /// The fixed roadmap over which the search is done.
  gls::datastructures::Graph mGraph;

  /// Source vertex.
  gls::datastructures::Vertex mStartVertex;

  /// Goal vertex.
  gls::datastructures::Vertex mGoalVertex;

};

} // namespace gls

#endif // GLS_GLS_HPP_
