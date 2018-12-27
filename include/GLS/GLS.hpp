/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_GLS_HPP_
#define GLS_GLS_HPP_

// STL headers
#include <exception>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/PathGeometric.h>

// GLS headers
#include "GLS/Datastructures/Graph.hpp"
#include "GLS/Datastructures/SearchQueue.hpp"
#include "GLS/Event/Event.hpp"
#include "GLS/Selector/Selector.hpp"

namespace gls {

enum TreeValidityStatus
{
  Valid,
  NotValid
};

/// The OMPL Planner class that implements the algorithm.
class GLS : public ompl::base::Planner
{
public:
  /// Constructor.
  /// \param[in] si The OMPL space information manager.
  explicit GLS(const ompl::base::SpaceInformationPtr& si);

  /// Destructor.
  ~GLS(void);

  /// Setup the planner.
  void setup() override;

  /// Set the problem definition and define the start, goal.
  /// \param[in] pdef OMPL Problem Definition.
  void setProblemDefinition(
      const ompl::base::ProblemDefinitionPtr& pdef) override;

  /// Solve the planning problem.
  /// \param[in] ptc OMPL Planning Termination Condition.
  ompl::base::PlannerStatus solve(
      const ompl::base::PlannerTerminationCondition& ptc);

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
  /// Returns edge between source and target vertices.
  gls::datastructures::Edge getEdge(
      gls::datastructures::Vertex, gls::datastructures::Vertex);

  /// Extends the search tree forwards.
  void extendSearchTree();

  /// Rewires the search tree when edge costs change.
  void rewireSearchTree();

  /// Evaluates the search tree when the extension pauses.
  TreeValidityStatus evaluateSearchTree();

  /// Return the path from source to target vertices.
  ompl::base::PathPtr constructSolution(const gls::datastructures::Vertex&, const gls::datastructures::Vertex&);

  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// SearchQueue representing the open list to extend.
  gls::datastructures::SearchQueue mExtendQueue;

  /// SearchQueue representing the vertices whose attirbutes need update.
  gls::datastructures::SearchQueue mUpdateQueue;

  /// SearchQueue representing the search tree that needs repairing.
  gls::datastructures::SearchQueue mRewireQueue;

  /// Event
  gls::event::EventPtr mEvent;

  /// Selector
  gls::selector::SelectorPtr mSelector;

  /// The fixed roadmap over which the search is done.
  gls::datastructures::Graph mGraph;

  /// Source vertex.
  gls::datastructures::Vertex mSourceVertex;

  /// Target vertex.
  gls::datastructures::Vertex mTargetVertex;
};

} // namespace gls

#endif // GLS_GLS_HPP_
