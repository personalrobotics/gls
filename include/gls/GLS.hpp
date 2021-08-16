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

// GLS headers. Include all the headers.
#include "gls/datastructures.hpp"
#include "gls/event.hpp"
#include "gls/io.hpp"
#include "gls/selector.hpp"
#include "gls/common.hpp"

namespace gls {

enum TreeValidityStatus { Valid, NotValid };

enum PlannerStatus { Solved, NotSolved };

typedef std::function<double(datastructures::StatePtr, datastructures::StatePtr)> HeuristicFunction;

/// The OMPL Planner class that implements the algorithm.
class GLS : public ompl::base::Planner {
public:
  /// Constructor.
  /// \param[in] si The OMPL space information manager.
  explicit GLS(const ompl::base::SpaceInformationPtr& si);

  /// Destructor.
  ~GLS(void);

  /// Setup the planner.
  void setup() override;

  /// If there are multiple goals set them. This will override the target state in pdef
  void setMultipleGoals(std::vector<datastructures::StatePtr> target_states);

  /// Set the problem definition and define the start, goal.
  /// \param[in] pdef OMPL Problem Definition.
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) override;

  /// Solve the planning problem.
  /// \param[in] ptc OMPL Planning Termination Condition.
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc);

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

  /// Set the connection radius of the graph.
  void setConnectionRadius(double radius);

  /// Get the connection radius of the graph.
  double getConnectionRadius();

  /// Set the collision checking resolution along the edge.
  void setCollisionCheckResolution(double resolution);

  /// Get the connection radius of the graph.
  double getCollisionCheckResolution();

  /// Set the roadmap. Loads the graph. Explicit only.
  void setRoadmap(std::string filename);

  /// Sets transition function for graph. Implicit only. 
  void setImplicit(datastructures::DiscFunc disc_function, datastructures::NeighborFunc transition_function, datastructures::NeighborFunc parent_function, datastructures::HashFunc hash_function, datastructures::InterpolateFunc interpolate_function=NULL);

  /// Set heuristic function, optional 
  void setHeuristic(HeuristicFunction heuristic);

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
  gls::datastructures::Edge getEdge(gls::datastructures::Vertex, gls::datastructures::Vertex);

  /// Returns the path from vertex to source.
  gls::datastructures::Path getPathToSource(gls::datastructures::Vertex);

  /// Returns true if the path to goal is collision-free.
  bool foundPathToGoal();

  /// Heuristic function.
  double getGraphHeuristic(gls::datastructures::Vertex v);

  /// Evaluates an edge for collision.
  gls::datastructures::CollisionStatus evaluateVertex(gls::datastructures::Vertex v);

  /// Evaluates an edge for collision.
  gls::datastructures::CollisionStatus evaluateEdge(const gls::datastructures::Edge& e);

  /// Extends the search tree forwards.
  void extendSearchTree();

  /// Updates the vertex properties in the search tree.
  void updateSearchTree();

  /// Rewires the search tree when edge costs change.
  void rewireSearchTree();

  /// Evaluates the search tree when the extension pauses.
  void evaluateSearchTree();

  /// Return the path from source to target vertices.
  ompl::base::PathPtr constructSolution(
      const gls::datastructures::Vertex&, const gls::datastructures::Vertex&);

  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// Boost roadmap representation.
  boost::shared_ptr<io::RoadmapFromFile<
      gls::datastructures::EGraph,
      gls::datastructures::VPStateMap,
      gls::datastructures::State,
      gls::datastructures::EPLengthMap>>
      mRoadmap;

  /// True if using implicit graph
  bool mImplicit = false;

  /// For reconstructing the full path, optional
  datastructures::InterpolateFunc mInterpolate;

  /// Heuristic function, optional
  HeuristicFunction mHeuristic = NULL;

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
  gls::datastructures::SearchQueue mExtendQueue;

  /// SearchQueue representing the vertices whose attirbutes need update.
  gls::datastructures::SearchQueue mUpdateQueue;

  /// SearchQueue representing the search tree that needs repairing.
  gls::datastructures::SearchQueue mRewireQueue;

  /// Set of vertices used for rewiring.
  std::set<gls::datastructures::Vertex> mRewireSet;

  /// Event
  gls::event::EventPtr mEvent;

  /// Selector
  gls::selector::SelectorPtr mSelector;

  /// The fixed roadmap over which the search is done.
  gls::datastructures::Graph mGraph;

  /// Number of nodes extended
  int mNumExtended = 0;

  /// Source vertex.
  gls::datastructures::Vertex mSourceVertex;

  /// Target vertex.
  gls::datastructures::Vertex mTargetVertex;

  /// Target vertices
  std::vector<gls::datastructures::Vertex> mTargetVertices = {};

  /// TODO (avk): Move these into PlannerStatus class.
  /// Number of Edge Evaluations.
  double mNumberOfEdgeEvaluations{0};

  /// Number of Edge Rewires.
  double mNumberOfEdgeRewires{0};
};

} // namespace gls

#endif // GLS_GLS_HPP_
