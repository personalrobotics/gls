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
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/PathGeometric.h>

#include "gls/datastructures/Types.hpp"

namespace gls {

/// The OMPL Planner class that implements the algorithm.
class GLS : public ompl::base::Planner {
 public:
  // ==============================================================================================
  /// Forward declaration of datastructure classes that belong to GLS.
  /// Vertex.
  class VertexProperties;
  /// Edge.
  class EdgeProperties;
  /// Roadmap Manager.
  template <class Graph, class VStateMap, class StateWrapper, class ELength>
  class RoadmapFromFile;
  /// Search Queue.
  class SearchQueue;
  /// The event.
  class Event;
  /// The selector.
  class Selector;
  /// The state.
  class State;

  // ==============================================================================================
  /// Type declarations.
  /// \brief State.
  using StatePtr = std::shared_ptr<State>;
  using ConstStatePtr = std::shared_ptr<const State>;
  /// \brief Graph properties.
  using Graph =
      boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                            VertexProperties, EdgeProperties>;
  /// \brief Search Queue.
  using SearchQueuePtr = std::shared_ptr<SearchQueue>;
  /// \brief Event.
  using EventPtr = std::shared_ptr<Event>;
  using ConstEventPtr = std::shared_ptr<const Event>;
  /// \brief Selector.
  using SelectorPtr = std::shared_ptr<Selector>;
  using ConstSelectorPtr = std::shared_ptr<const Selector>;

  // ==============================================================================================
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
  void setEvent(EventPtr event);

  /// Returns the event used by the algorithm.
  ConstEventPtr getEvent() const;

  /// Set the selector to be used by GLS.
  /// \param[in] selector Selector that defines the evaluation strategy.
  void setSelector(SelectorPtr selector);

  /// Returns the selector used by the algorithm.
  ConstSelectorPtr getSelector() const;

  /// Set the connection radius of the graph.
  void setConnectionRadius(double radius);

  /// Get the connection radius of the graph.
  double getConnectionRadius();

  /// Set the collision checking resolution along the edge.
  void setCollisionCheckResolution(double resolution);

  /// Get the connection radius of the graph.
  double getCollisionCheckResolution();

  /// Set the roadmap. Loads the graph.
  void setRoadmap(std::string filename);

  /// Set the best path cost.
  void setBestPathCost(double cost);

  /// Get the best path cost.
  double getBestPathCost();

  /// Set status of the planner.
  void setPlannerStatus(PlannerStatus);

  /// Get status of the planner.
  PlannerStatus getPlannerStatus();

  /// Get the number of edges evaluated.
  double getNumberOfEdgeEvaluations();

  /// Get the number of edges rewired.
  double getNumberOfEdgeRewires();

 private:
  /// Adds source and target vertices, and relevant edges to \c mGraph.
  /// Sets up the event and the selector.
  void setupPreliminaries();

  /// Returns edge between source and target vertices.
  Edge getEdge(Vertex, Vertex);

  /// Returns the path from vertex to source.
  Path getPathToSource(Vertex);

  /// Returns true if the path to goal is collision-free.
  bool foundPathToGoal();

  /// Heuristic function.
  double getGraphHeuristic(Vertex v);

  /// Evaluates an edge for collision.
  CollisionStatus evaluateVertex(Vertex v);

  /// Evaluates an edge for collision.
  CollisionStatus evaluateEdge(const Edge& e);

  /// Rewires the vertex to the best parent not in subtree being rewired.
  void rewireToBestParent(const Vertex& vertex);

  /// Rewires to children if they find /c vertex to be a better parent.
  void rewireToChildren(const Vertex& vertex);

  /// Extends the search tree forwards.
  void extendSearchTree();

  /// Updates the vertex properties in the search tree.
  void updateSearchTree();

  /// Rewires the search tree when edge costs change.
  void rewireSearchTree();

  /// Evaluates the search tree when the extension pauses.
  void evaluateSearchTree();

  /// Return the path from source to target vertices.
  ompl::base::PathPtr constructSolution(const Vertex&, const Vertex&);

  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// Connection radius in the graph.
  double mConnectionRadius;

  /// Collision checking resolution for the edge.
  double mCollisionCheckResolution;

  /// Boolean denoting if the graph has been setup.
  bool mGraphSetup{false};

  /// Filename containing the roadmap.
  double mBestPathCost{0};

  /// Flag to check if the planner succeeded.
  PlannerStatus mPlannerStatus{PlannerStatus::NotSolved};

  /// Flag to check the validity of the search tree.
  TreeValidityStatus mTreeValidityStatus{TreeValidityStatus::Valid};

  /// SearchQueue representing the open list to extend.
  SearchQueuePtr mExtendQueue{nullptr};

  /// SearchQueue representing the vertices whose attirbutes need update.
  SearchQueuePtr mUpdateQueue{nullptr};

  /// SearchQueue representing the search tree that needs repairing.
  SearchQueuePtr mRewireQueue{nullptr};

  /// Set of vertices used for rewiring.
  std::set<Vertex> mRewireSet;

  /// Event
  EventPtr mEvent;

  /// Selector
  SelectorPtr mSelector;

  /// The fixed roadmap over which the search is done.
  Graph mGraph;

  /// Source vertex.
  Vertex mSourceVertex;

  /// Target vertex.
  Vertex mTargetVertex;

  /// TODO (avk): Move these into PlannerStatus class.
  /// Number of Edge Evaluations.
  double mNumberOfEdgeEvaluations{0};

  /// Number of Edge Rewires.
  double mNumberOfEdgeRewires{0};
};

}  // namespace gls

#endif  // GLS_GLS_HPP_
