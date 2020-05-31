/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/GLS.hpp"

#include <assert.h>

#include <boost/graph/connected_components.hpp>
#include <cmath>
#include <iostream>
#include <set>

#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/SearchQueue.hpp"
#include "gls/datastructures/State.hpp"
#include "gls/event/Event.hpp"
#include "gls/io/RoadmapManager.hpp"
#include "gls/selector/Selector.hpp"

namespace gls {

GLS::GLS(const ompl::base::SpaceInformationPtr& si)
    : ompl::base::Planner(si, "GLS"), mSpace(si->getStateSpace()) {
  // Set default values for data members.
}

GLS::~GLS() {
  // Do nothing.
}

// ============================================================================
void GLS::setup() {
  // Check if already setup.
  if (static_cast<bool>(ompl::base::Planner::setup_)) return;

  // Mark the planner to have been setup.
  ompl::base::Planner::setup();

  // Check if the graph has been setup.
  if (!mGraphSetup) {
    std::invalid_argument("Graph has not been provided.");
  }

  // TODO (avk): If the graph is not provided, use implicit representation
  // for the edges using the NearestNeighbor representation.
  // Check if roadmap has been provided.

  OMPL_INFORM("Planner has been setup.");
}

// ============================================================================
void GLS::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) {
  // Make sure we setup the planner first.
  if (!static_cast<bool>(ompl::base::Planner::setup_)) {
    setup();
  }

  // Mark the planner's problem to be defined.
  ompl::base::Planner::setProblemDefinition(pdef);

  setupPreliminaries();
  OMPL_INFORM("Problem Definition has been setup.");
}

// ============================================================================
void GLS::setupPreliminaries() {
  // Issue a warning if mConnectionRadius = 0.

  // TODO (avk): Should I kill these pointers manually?
  StatePtr sourceState(new State(mSpace));
  mSpace->copyState(sourceState->getOMPLState(), pdef_->getStartState(0));

  StatePtr targetState(new State(mSpace));
  mSpace->copyState(targetState->getOMPLState(),
                    pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

  // Add start and goal vertices to the graph
  mSourceVertex = boost::add_vertex(mGraph);
  mGraph[mSourceVertex].setState(sourceState);

  mTargetVertex = boost::add_vertex(mGraph);
  mGraph[mTargetVertex].setState(targetState);

  // Assign default values.
  mGraph[mSourceVertex].setCostToCome(0);
  mGraph[mSourceVertex].setHeuristic(getGraphHeuristic(mSourceVertex));
  mGraph[mSourceVertex].setVisitStatus(VisitStatus::NotVisited);
  mGraph[mSourceVertex].setCollisionStatus(CollisionStatus::Free);
  mGraph[mSourceVertex].setParent(mSourceVertex);

  mGraph[mTargetVertex].setCostToCome(std::numeric_limits<double>::max());
  mGraph[mTargetVertex].setHeuristic(0);
  mGraph[mTargetVertex].setVisitStatus(VisitStatus::NotVisited);
  mGraph[mTargetVertex].setCollisionStatus(CollisionStatus::Free);

  // TODO (AVK): Make this kNN + R-disc. Additionally join the start and goal.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    double sourceDistance = mSpace->distance(
        mGraph[*vi].getState()->getOMPLState(), sourceState->getOMPLState());
    double targetDistance = mSpace->distance(
        mGraph[*vi].getState()->getOMPLState(), targetState->getOMPLState());

    if (sourceDistance < mConnectionRadius) {
      if (mSourceVertex == *vi) continue;

      std::pair<Edge, bool> newEdge =
          boost::add_edge(mSourceVertex, *vi, mGraph);

      mGraph[newEdge.first].setLength(sourceDistance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);
      assert(newEdge.second);
    }

    if (targetDistance < mConnectionRadius) {
      if (mTargetVertex == *vi) continue;

      std::pair<Edge, bool> newEdge =
          boost::add_edge(*vi, mTargetVertex, mGraph);
      mGraph[newEdge.first].setLength(targetDistance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);
      assert(newEdge.second);
    }
  }

  // Additionally connect the source and target with a straight line to snap.
  std::pair<Edge, bool> newEdge =
      boost::add_edge(mSourceVertex, mTargetVertex, mGraph);
  mGraph[newEdge.first].setLength(mSpace->distance(
      sourceState->getOMPLState(), targetState->getOMPLState()));
  mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
  mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);

  // Setup the event.
  mEvent->setup(&mGraph, mSourceVertex, mTargetVertex);

  // Setup the selector.
  mSelector->setup(&mGraph);

  // Provide the search queue access to the graph.
  mExtendQueue = std::make_shared<SearchQueue>("Extend");
  mUpdateQueue = std::make_shared<SearchQueue>("Update");
  mRewireQueue = std::make_shared<SearchQueue>("Rewire");
  mExtendQueue->setGraph(&mGraph);
  mUpdateQueue->setGraph(&mGraph);
  mRewireQueue->setGraph(&mGraph);
}

// ============================================================================
void GLS::clear() {
  // Call the base clear
  ompl::base::Planner::clear();

  // Clear the queues.
  mExtendQueue->clear();
  mRewireQueue->clear();
  mUpdateQueue->clear();
  mRewireSet.clear();

  // Reset the vertices and edges.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    mGraph[*vi].setCostToCome(std::numeric_limits<double>::max());
    mGraph[*vi].setHeuristic(std::numeric_limits<double>::max());
    mGraph[*vi].removeAllChildren();
    mGraph[*vi].setVisitStatus(VisitStatus::NotVisited);
    mGraph[*vi].setCollisionStatus(CollisionStatus::Free);
  }

  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei) {
    mGraph[*ei].setEvaluationStatus(EvaluationStatus::NotEvaluated);
    mGraph[*ei].setCollisionStatus(CollisionStatus::Free);
  }

  // Remove edges between source, target to other vertices.
  clear_vertex(mSourceVertex, mGraph);
  clear_vertex(mTargetVertex, mGraph);

  // Remove the vertices themselves.
  remove_vertex(mSourceVertex, mGraph);
  remove_vertex(mTargetVertex, mGraph);

  setBestPathCost(0);
  mNumberOfEdgeEvaluations = 0;
  mNumberOfEdgeRewires = 0;
  mTreeValidityStatus = TreeValidityStatus::Valid;
  mPlannerStatus = PlannerStatus::NotSolved;

  // TODO(avk): Clear the selector and event.

  OMPL_INFORM("Cleared Everything");
}

// ============================================================================
ompl::base::PlannerStatus GLS::solve(
    const ompl::base::PlannerTerminationCondition& /*ptc*/) {
  // TODO (avk): Use ptc to terminate the search.

  // Return if source or target are in collision.
  if (evaluateVertex(mSourceVertex) == CollisionStatus::Collision) {
    OMPL_INFORM("Start State is invalid.");
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (evaluateVertex(mTargetVertex) == CollisionStatus::Collision) {
    OMPL_INFORM("Goal State is invalid.");
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  // Add the source vertex to the search tree with zero cost-to-come.
  mGraph[mSourceVertex].setVisitStatus(VisitStatus::Visited);
  mEvent->updateVertexProperties(mSourceVertex);

  assert(mExtendQueue->isEmpty());
  mExtendQueue->enqueueVertex(mSourceVertex,
                              mGraph[mSourceVertex].getEstimatedTotalCost());

  // Run in loop.
  while (!mExtendQueue->isEmpty()) {
    // Extend the tree till the event is triggered.
    extendSearchTree();

    // Evaluate the extended search tree.
    evaluateSearchTree();

    // If the plan is successful, return.
    if (mPlannerStatus == PlannerStatus::Solved) {
      break;
    }
  }

  if (mPlannerStatus == PlannerStatus::Solved) {
    setBestPathCost(mGraph[mTargetVertex].getCostToCome());
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    OMPL_INFORM("Number of Edges Rewired:     %f", mNumberOfEdgeRewires);
    OMPL_INFORM("Number of Edges Evaluated:   %f", mNumberOfEdgeEvaluations);
    OMPL_INFORM("Cost of goal:                %f", mBestPathCost);
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  } else {
    OMPL_INFORM("No Solution Found.");
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}

// ============================================================================
void GLS::setEvent(EventPtr event) { mEvent = event; }

// ============================================================================
GLS::ConstEventPtr GLS::getEvent() const { return mEvent; }

// ============================================================================
void GLS::setSelector(SelectorPtr selector) { mSelector = selector; }

// ============================================================================
GLS::ConstSelectorPtr GLS::getSelector() const { return mSelector; }

// ============================================================================
Edge GLS::getEdge(Vertex u, Vertex v) {
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
Path GLS::getPathToSource(Vertex u) {
  Path pathToSource;
  while (u != mSourceVertex) {
    pathToSource.emplace_back(u);
    u = mGraph[u].getParent();
  }
  pathToSource.emplace_back(mSourceVertex);
  return pathToSource;
}

// ============================================================================
bool GLS::foundPathToGoal() {
  if (mGraph[mTargetVertex].getVisitStatus() == VisitStatus::NotVisited) {
    return false;
  }

  Path pathToGoal = getPathToSource(mTargetVertex);
  for (std::size_t i = 0; i < pathToGoal.size() - 1; ++i) {
    Edge e = getEdge(pathToGoal[i + 1], pathToGoal[i]);
    if (mGraph[e].getEvaluationStatus() == EvaluationStatus::NotEvaluated) {
      return false;
    }
  }
  return true;
}

// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double GLS::getGraphHeuristic(Vertex v) {
  double heuristic =
      mSpace->distance(mGraph[mTargetVertex].getState()->getOMPLState(),
                       mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
void GLS::setConnectionRadius(double radius) { mConnectionRadius = radius; }

// ============================================================================
double GLS::getConnectionRadius() { return mConnectionRadius; }

// ============================================================================
void GLS::setCollisionCheckResolution(double resolution) {
  mCollisionCheckResolution = resolution;
}

// ============================================================================
double GLS::getCollisionCheckResolution() { return mCollisionCheckResolution; }

// ============================================================================
void GLS::setRoadmap(std::string filename) {
  if (filename == "")
    std::invalid_argument("Roadmap Filename cannot be empty!");

  // Load the graph.
  auto roadmap =
      boost::shared_ptr<RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>>(
          new RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>(mSpace,
                                                                     filename));

  roadmap->generate(mGraph, get(&GLS::VertexProperties::mState, mGraph),
                    get(&GLS::EdgeProperties::mLength, mGraph));

  // Mark the graph to have been setup.
  mGraphSetup = true;
}

// ============================================================================
void GLS::setBestPathCost(double cost) { mBestPathCost = cost; }

// ============================================================================
double GLS::getBestPathCost() { return mBestPathCost; }

// ============================================================================
void GLS::setPlannerStatus(PlannerStatus status) { mPlannerStatus = status; }

// ============================================================================
PlannerStatus GLS::getPlannerStatus() { return mPlannerStatus; }

// ============================================================================
double GLS::getNumberOfEdgeEvaluations() { return mNumberOfEdgeEvaluations; }

// ============================================================================
double GLS::getNumberOfEdgeRewires() { return mNumberOfEdgeRewires; }

// ============================================================================
CollisionStatus GLS::evaluateVertex(Vertex v) {
  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  // Evaluate the state.
  auto state = mGraph[v].getState()->getOMPLState();
  if (!validityChecker->isValid(state)) {
    return CollisionStatus::Collision;
  }
  return CollisionStatus::Free;
}

// ============================================================================
CollisionStatus GLS::evaluateEdge(const Edge& e) {
  mNumberOfEdgeEvaluations++;

  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  // Collision check the start and goal.
  Vertex startVertex = source(e, mGraph);
  Vertex endVertex = target(e, mGraph);
  std::cout << startVertex << " " << endVertex << std::endl;

  auto startState = mGraph[startVertex].getState()->getOMPLState();
  auto endState = mGraph[endVertex].getState()->getOMPLState();

  // Evaluate Start and End States.
  if (!validityChecker->isValid(startState)) {
    mGraph[startVertex].setCollisionStatus(CollisionStatus::Collision);
    return CollisionStatus::Collision;
  }

  if (!validityChecker->isValid(endState)) {
    mGraph[endVertex].setCollisionStatus(CollisionStatus::Collision);
    return CollisionStatus::Collision;
  }

  // Evaluate the state in between.
  int maxSteps = 1.0 / mCollisionCheckResolution;
  for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier) {
    double interpolationStep = mCollisionCheckResolution * multiplier;
    assert(interpolationStep <= 1);
    StatePtr midVertex(new State(mSpace));
    mSpace->interpolate(startState, endState, interpolationStep,
                        midVertex->getOMPLState());

    if (!validityChecker->isValid(midVertex->getOMPLState())) {
      return CollisionStatus::Collision;
    }
  }

  // Edge is collision-free.
  return CollisionStatus::Free;
}

// ============================================================================
void GLS::extendSearchTree() {
  // Ideally extend search tree should not be called when the queue is empty.
  assert(!mExtendQueue->isEmpty());

  while (!mExtendQueue->isEmpty()) {
    mExtendQueue->printQueue();

    // Check if the popping the top vertex triggers the event.
    if (mEvent->isTriggered(mExtendQueue->getTopVertex())) {
      break;
    }

    // Pop the top vertex in the queue to extend the search tree.
    Vertex u = mExtendQueue->popTopVertex();
    std::cout << "Popping out " << u << std::endl;
    // std::cin.get();

    // The vertex being extended should have been marked visited.
    assert(mGraph[u].getVisitStatus() == VisitStatus::Visited);

    // If the vertex has been marked to be in collision, do not extend.
    if (mGraph[u].getCollisionStatus() == CollisionStatus::Collision) {
      continue;
    }

    // Get the neighbors and extend.
    // TODO (avk): Have a wrapper in case the implicit vs explicit.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end;
         ++ni) {
      Vertex v = *ni;
      std::cout << "Possible neighbor " << v << std::endl;

      // If the successor has been previously marked to be in collision,
      // continue to the next successor.
      if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision) {
        continue;
      }

      // Enforce prevention of loops.
      // TODO(avk): If the child was rewired to parent in rewire, should ignore
      if (v == mGraph[u].getParent()) {
        continue;
      }

      // Never come back to the source.
      if (v == mSourceVertex) {
        continue;
      }

      // Get the edge between the two vertices.
      Edge uv = getEdge(u, v);

      // If the edge has been previously marked to be in collision,
      // continue to the next successor.
      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Collision) {
        continue;
      }

      double edgeLength = mGraph[uv].getLength();
      if (mGraph[v].getVisitStatus() == VisitStatus::NotVisited) {
        assert(v != mSourceVertex);
        mGraph[v].setVisitStatus(VisitStatus::Visited);
        assert(!mGraph[v].inSearchQueue());
      } else {
        double oldCostToCome = mGraph[v].getCostToCome();
        double newCostToCome = mGraph[u].getCostToCome() + edgeLength;

        // Use the parent ID to break ties.
        Vertex previousParent = mGraph[v].getParent();

        // If the previous cost-to-come is lower, continue.
        if (oldCostToCome < newCostToCome) {
          std::cout << __LINE__ << std::endl;
          continue;
        }

        // Tie-Breaking Rule
        if (oldCostToCome == newCostToCome) {
          if (previousParent < u) {
            std::cout << __LINE__ << std::endl;
            continue;
          }
        }

        // The new cost is lower, make necessary updates.
        // Remove from previous siblings.
        mGraph[previousParent].removeChild(v);

        // Remove the previous version of the vertex from possible queues.
        if (mGraph[v].inSearchQueue()) {
          mExtendQueue->dequeueVertex(v);
        }

        // Cascade the updates to all the descendents.
        // Replace this with getDescendents() function.
        std::vector<Vertex> subtree = {v};
        while (!subtree.empty()) {
          auto iterT = subtree.rbegin();
          if (*iterT == 82) {
            std::cout << "Clearning 82s children" << std::endl;
          }
          std::set<Vertex>& children = mGraph[*iterT].getChildren();
          subtree.pop_back();

          for (auto child : children) {
            mGraph[child].setVisitStatus(VisitStatus::NotVisited);
            mGraph[child].setCostToCome(std::numeric_limits<double>::max());
            mGraph[child].setParent(child);
            subtree.emplace_back(child);
            if (mGraph[child].inSearchQueue()) {
              mExtendQueue->dequeueVertex(child);
            }
          }
          children.clear();
        }
      }

      // Update the successor's properties.
      mGraph[v].setParent(u);
      mGraph[v].setCostToCome(mGraph[u].getCostToCome() + edgeLength);
      mGraph[v].setHeuristic(getGraphHeuristic(v));

      // Update the vertex property associated with the event.
      mEvent->updateVertexProperties(v);

      // Add it to its new siblings
      mGraph[u].addChild(v);

      std::cout << "Adding " << v << " to the extend queue" << std::endl;
      mExtendQueue->enqueueVertex(v, mGraph[v].getEstimatedTotalCost());
      mExtendQueue->printQueue();
      // std::cin.get();
    }
  }
}

// ============================================================================
void GLS::updateSearchTree() {
  if (mTreeValidityStatus == TreeValidityStatus::Valid) {
    // Update the vertex properties of the entire search tree.
    mEvent->updateVertexProperties(mUpdateQueue);
    assert(mUpdateQueue->isEmpty());
  } else {
    // Rewire the search tree.
    rewireSearchTree();

    // With successful rewire, mark the tree to be valid again.
    mTreeValidityStatus = TreeValidityStatus::Valid;
  }
}

// ============================================================================
void GLS::rewireSearchTree() {
  // Mark all the vertices to be rewired.
  std::vector<Vertex> repairVertices;
  while (!mRewireQueue->isEmpty()) {
    const auto vertex = mRewireQueue->popTopVertex();

    if (vertex == 82) {
      std::cout << "Caught 82" << std::endl;
    }

    repairVertices.push_back(vertex);
    mGraph[vertex].setInRepair(true);
    auto& children = mGraph[vertex].getChildren();
    for (const auto& child : children) {
      if (mGraph[child].inSearchQueue()) {
        mExtendQueue->dequeueVertex(child);
      }
      mRewireQueue->enqueueVertex(child, mGraph[child].getCostToCome());
    }
    mGraph[vertex].removeAllChildren();
  }

  // Find the best parent.
  for (const auto& vertex : repairVertices) {
    const auto previousCostToCome = mGraph[vertex].getCostToCome();
    rewireToBestParent(vertex);
    // TODO(avk): If the vertex has found a good parent, and the cost has not
    // changed by much, terminate the rewire process.
    mRewireQueue->enqueueVertex(vertex, mGraph[vertex].getCostToCome());
  }
  mRewireQueue->printQueue();
  // std::cin.get();

  // Process the entire subtree.
  while (!mRewireQueue->isEmpty()) {
    const auto vertex = mRewireQueue->popTopVertex();
    mGraph[vertex].setInRepair(false);

    // If the vertex has already found a good parent, rewiring is complete.
    Vertex parent = mGraph[vertex].getParent();
    if (parent != vertex) {
      std::cout << vertex << " rewired to " << parent << " whose parent is "
                << mGraph[parent].getParent() << " "
                << mGraph[parent].getCostToCome() << std::endl;
      // std::cin.get();
      // Let the parent know of its new child.
      mGraph[parent].addChild(vertex);
      if (parent == 82) {
        std::cout << "caught 82" << std::endl;
      }

      // If the vertex triggers the event, simply add to extend queue.
      if (mEvent->isTriggered(vertex)) {
        mExtendQueue->enqueueVertex(vertex,
                                    mGraph[vertex].getEstimatedTotalCost());
      } else {
        // See if it can be a better parent to its children who are in repair.
        rewireToChildren(vertex);
      }
    } else {
      std::cout << vertex << " could not get rewired" << std::endl;
      mGraph[vertex].setVisitStatus(VisitStatus::NotVisited);
    }
  }
}

// ============================================================================
void GLS::rewireToBestParent(const Vertex& vertex) {
  // Track the best parent and the cost to come.
  double bestCostToCome = std::numeric_limits<double>::max();
  Vertex bestParent;

  NeighborIter ni, ni_end;
  for (boost::tie(ni, ni_end) = adjacent_vertices(vertex, mGraph); ni != ni_end;
       ++ni) {
    Vertex v = *ni;

    // If the predecessor is in repair, ignore.
    if (mGraph[v].getInRepair()) {
      continue;
    }

    // If the predecessor is a goal vertex, ignore.
    if (v == mTargetVertex) {
      continue;
    }

    // If the predecessor has not been visited yet, ignore.
    // if (mGraph[v].getVisitStatus() == VisitStatus::NotVisited) {
    // continue;
    // }
    assert(vertex != mGraph[vertex].getParent());

    // If the vertex is in search queue (either), ignore.
    // if (mGraph[v].inSearchQueue()) {
    //   continue;
    // }

    // If the predecessor was marked in collision, or the edge, ignore.
    if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision) {
      continue;
    }
    Edge edge = getEdge(v, vertex);
    if (mGraph[edge].getCollisionStatus() == CollisionStatus::Collision) {
      continue;
    }
    double potentialCostToCome =
        mGraph[v].getCostToCome() + mGraph[edge].getLength();
    if (potentialCostToCome < bestCostToCome) {
      bestCostToCome = potentialCostToCome;
      bestParent = v;
    }
  }
  // We looked at all predecessors, did we find a parent?
  mGraph[vertex].setCostToCome(bestCostToCome);
  if (bestCostToCome != std::numeric_limits<double>::max()) {
    mGraph[vertex].setParent(bestParent);
  } else {
    mGraph[vertex].setParent(vertex);
  }
}

// ============================================================================
void GLS::rewireToChildren(const Vertex& vertex) {
  NeighborIter ni, ni_end;
  for (boost::tie(ni, ni_end) = adjacent_vertices(vertex, mGraph); ni != ni_end;
       ++ni) {
    Vertex v = *ni;

    // If the successor is not in repair, ignore.
    if (!mGraph[v].getInRepair()) {
      continue;
    }

    // Never come back to the source.
    if (v == mSourceVertex) {
      continue;
    }

    // Get the edge between the two vertices.
    Edge uv = getEdge(vertex, v);

    // If the edge has been previously marked to be in collision,
    // continue to the next successor.
    if (mGraph[uv].getCollisionStatus() == CollisionStatus::Collision) {
      continue;
    }

    double potentialCostToCome =
        mGraph[vertex].getCostToCome() + mGraph[uv].getLength();
    if (potentialCostToCome < mGraph[v].getCostToCome()) {
      // Update the vertex.
      mGraph[v].setCostToCome(potentialCostToCome);
      mGraph[v].setParent(vertex);
      // Assert that the vertex is in the rewire queue.
      assert(mGraph[v].inSearchQueue());
      mRewireQueue->dequeueVertex(v);
      mRewireQueue->enqueueVertex(v, potentialCostToCome);
    }
  }
}

// ============================================================================
void GLS::evaluateSearchTree() {
  if (mExtendQueue->isEmpty()) {
    return;
  }

  Vertex bestVertex = mExtendQueue->getTopVertex();
  Edge edgeToEvaluate =
      mSelector->selectEdgeToEvaluate(getPathToSource(bestVertex));

  Vertex u = source(edgeToEvaluate, mGraph);
  Vertex v = target(edgeToEvaluate, mGraph);
  Edge uv = getEdge(u, v);

  // Assume that the selector might return edges already evaluated.
  assert(mGraph[uv].getEvaluationStatus() != EvaluationStatus::Evaluated);

  // Evaluate the edge.
  mGraph[uv].setEvaluationStatus(EvaluationStatus::Evaluated);
  if (evaluateEdge(uv) == CollisionStatus::Free) {
    // Set the edge collision status.
    mGraph[uv].setCollisionStatus(CollisionStatus::Free);

    // Populate the queue to update the search tree.
    mUpdateQueue->enqueueVertex(v, mGraph[v].getCostToCome());
  } else {
    mGraph[uv].setCollisionStatus(CollisionStatus::Collision);
    mTreeValidityStatus = TreeValidityStatus::NotValid;

    // Let the old parent know that the child has been removed.
    mGraph[u].removeChild(v);

    // Remove v from search queue if it exists there and add to rewire.
    if (mGraph[v].inSearchQueue()) {
      mExtendQueue->dequeueVertex(v);
    }
    mRewireQueue->enqueueVertex(v, mGraph[v].getCostToCome());
  }

  if (bestVertex == mTargetVertex &&
      mTreeValidityStatus == TreeValidityStatus::Valid) {
    if (foundPathToGoal()) {
      // Planning problem has been solved!
      mPlannerStatus = PlannerStatus::Solved;
    }
  }

  // Based on the evaluation, update the search tree.
  updateSearchTree();
}

// ============================================================================
ompl::base::PathPtr GLS::constructSolution(const Vertex& source,
                                           const Vertex& target) {
  ompl::geometric::PathGeometric* path =
      new ompl::geometric::PathGeometric(si_);
  Vertex v = target;

  while (v != source) {
    path->append(mGraph[v].getState()->getOMPLState());
    v = mGraph[v].getParent();
  }

  if (v == source) {
    path->append(mGraph[source].getState()->getOMPLState());
  }
  path->reverse();
  return ompl::base::PathPtr(path);
}

}  // namespace gls
