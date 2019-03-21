/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/GLS.hpp"

#include <cmath>     // pow, sqrt
#include <iostream>  // std::invalid_argument
#include <set>       // std::set
#include <assert.h>  // debug

#include <boost/graph/connected_components.hpp> // connected_components

using gls::datastructures::CollisionStatus;
using gls::datastructures::Edge;
using gls::datastructures::EdgeIter;
using gls::datastructures::EdgeProperties;
using gls::datastructures::EPLengthMap;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Graph;
using gls::datastructures::Path;
using gls::datastructures::NeighborIter;
using gls::datastructures::State;
using gls::datastructures::StatePtr;
using gls::datastructures::Vertex;
using gls::datastructures::VertexIter;
using gls::datastructures::VertexProperties;
using gls::datastructures::VisitStatus;
using gls::datastructures::VPStateMap;
using gls::event::ConstEventPtr;
using gls::selector::ConstSelectorPtr;
using gls::event::EventPtr;
using gls::selector::SelectorPtr;

namespace gls {

GLS::GLS(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "GLS"), mSpace(si->getStateSpace())
{
  // Set default values for data members.
}

GLS::~GLS()
{
  // Do nothing.
}

// ============================================================================
void GLS::setup()
{
  // Check if already setup.
  if (static_cast<bool>(ompl::base::Planner::setup_))
    return;

  // Mark the planner to have been setup.
  ompl::base::Planner::setup();

  // Check if the graph has been setup.
  if (!mGraphSetup)
    std::invalid_argument("Graph has not been provided.");

  // TODO (avk): If the graph is not provided, use implicit representation
  // for the edges using the NearestNeighbor representation.
  // Check if roadmap has been provided.

  OMPL_INFORM("Planner has been setup.");
}

// ============================================================================
void GLS::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef)
{
  // Make sure we setup the planner first.
  if (!static_cast<bool>(ompl::base::Planner::setup_))
  {
    setup();
  }

  // Mark the planner's problem to be defined.
  ompl::base::Planner::setProblemDefinition(pdef);

  setupPreliminaries();
  OMPL_INFORM("Problem Definition has been setup.");
}

// ============================================================================
void GLS::setupPreliminaries()
{
  // Issue a warning if mConnectionRadius = 0.

  StatePtr sourceState(new gls::datastructures::State(mSpace));
  mSpace->copyState(sourceState->getOMPLState(), pdef_->getStartState(0));

  StatePtr targetState(new gls::datastructures::State(mSpace));
  mSpace->copyState(
      targetState->getOMPLState(),
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
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
  {
    double sourceDistance = mSpace->distance(
        mGraph[*vi].getState()->getOMPLState(), sourceState->getOMPLState());
    double targetDistance = mSpace->distance(
        mGraph[*vi].getState()->getOMPLState(), targetState->getOMPLState());

    if (sourceDistance < mConnectionRadius)
    {
      if (mSourceVertex == *vi)
        continue;

      std::pair<Edge, bool> newEdge
          = boost::add_edge(mSourceVertex, *vi, mGraph);

      mGraph[newEdge.first].setLength(sourceDistance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);
      assert(newEdge.second);
    }

    if (targetDistance < mConnectionRadius)
    {
      if (mTargetVertex == *vi)
        continue;

      std::pair<Edge, bool> newEdge
          = boost::add_edge(mTargetVertex, *vi, mGraph);
      mGraph[newEdge.first].setLength(targetDistance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);
      assert(newEdge.second);
    }
  }

  // Additionally connect the source and target with a straight line to snap.
  std::pair<Edge, bool> newEdge
      = boost::add_edge(mSourceVertex, mTargetVertex, mGraph);
  mGraph[newEdge.first].setLength(
      mSpace->distance(
          sourceState->getOMPLState(), targetState->getOMPLState()));
  mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
  mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);

  // Setup the event.
  mEvent->setup(&mGraph, mSourceVertex, mTargetVertex);

  // Setup the selector.
  mSelector->setup(&mGraph);
}

// ============================================================================
void GLS::clear()
{
  // Call the base clear
  ompl::base::Planner::clear();

  // Clear the queues.
  mExtendQueue.clear();
  mRewireQueue.clear();
  mUpdateQueue.clear();
  mRewireSet.clear();
  assert(mExtendQueue.isEmpty());
  assert(mRewireQueue.isEmpty());
  assert(mUpdateQueue.isEmpty());

  // Reset the vertices and edges.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
  {
    mGraph[*vi].setCostToCome(std::numeric_limits<double>::max());
    mGraph[*vi].setHeuristic(std::numeric_limits<double>::max());
    mGraph[*vi].removeAllChildren();
    mGraph[*vi].setVisitStatus(VisitStatus::NotVisited);
    mGraph[*vi].setCollisionStatus(CollisionStatus::Free);
  }

  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei)
  {
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
    const ompl::base::PlannerTerminationCondition& /*ptc*/)
{
  // TODO (avk): Use ptc to terminate the search.

  // Return if source or target are in collision.
  if (evaluateVertex(mSourceVertex) == CollisionStatus::Collision)
    return ompl::base::PlannerStatus::INVALID_START;

  if (evaluateVertex(mTargetVertex) == CollisionStatus::Collision)
    return ompl::base::PlannerStatus::INVALID_GOAL;

  // Add the source vertex to the search tree with zero cost-to-come.
  mGraph[mSourceVertex].setVisitStatus(VisitStatus::Visited);
  mEvent->updateVertexProperties(mSourceVertex);

  assert(mExtendQueue.isEmpty());
  auto previousSize = mExtendQueue.getSize();
  mExtendQueue.addVertexWithValue(mSourceVertex, mGraph[mSourceVertex].getEstimatedTotalCost());
  auto currentSize = mExtendQueue.getSize();
  assert(currentSize - previousSize == 1);

  // Run in loop.
  while (!mExtendQueue.isEmpty())
  {
    // Extend the tree till the event is triggered.
    extendSearchTree();

    // Evaluate the extended search tree.
    evaluateSearchTree();

    // If the plan is successful, return.
    if (mPlannerStatus == PlannerStatus::Solved)
      break;
  }

  if (mPlannerStatus == PlannerStatus::Solved)
  {
    setBestPathCost(mGraph[mTargetVertex].getCostToCome());
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
  else
  {
    Edge uv = getEdge(mSourceVertex, mTargetVertex);
    assert(mGraph[uv].getEvaluationStatus() == EvaluationStatus::Evaluated);
    assert(mGraph[uv].getCollisionStatus() == CollisionStatus::Collision);
    OMPL_INFORM("No Solution Found.");
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}

// ============================================================================
void GLS::setEvent(EventPtr event)
{
  mEvent = event;
}

// ============================================================================
ConstEventPtr GLS::getEvent() const
{
  return mEvent;
}

// ============================================================================
void GLS::setSelector(SelectorPtr selector)
{
  mSelector = selector;
}

// ============================================================================
ConstSelectorPtr GLS::getSelector() const
{
  return mSelector;
}

// ============================================================================
Edge GLS::getEdge(Vertex u, Vertex v)
{
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
Path GLS::getPathToSource(Vertex u)
{
  Path pathToSource;
  while (u != mSourceVertex)
  {
    pathToSource.emplace_back(u);
    u = mGraph[u].getParent();
  }
  pathToSource.emplace_back(mSourceVertex);
  return pathToSource;
}

// ============================================================================
bool GLS::foundPathToGoal()
{
  if (mGraph[mTargetVertex].getVisitStatus() == VisitStatus::NotVisited)
    return false;

  Path pathToGoal = getPathToSource(mTargetVertex);
  for (std::size_t i = 0; i < pathToGoal.size() - 1; ++i)
  {
    Edge e = getEdge(pathToGoal[i+1], pathToGoal[i]);
    if (mGraph[e].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
      return false;
  }

  return true;
}

// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double GLS::getGraphHeuristic(Vertex v)
{
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(),
      mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
void GLS::setConnectionRadius(double radius)
{
  mConnectionRadius = radius;
}

// ============================================================================
double GLS::getConnectionRadius()
{
  return mConnectionRadius;
}

// ============================================================================
void GLS::setCollisionCheckResolution(double resolution)
{
  mCollisionCheckResolution = resolution;
}

// ============================================================================
double GLS::getCollisionCheckResolution()
{
  return mCollisionCheckResolution;
}

// ============================================================================
void GLS::setRoadmap(std::string filename)
{
  if (filename == "")
    std::invalid_argument("Roadmap Filename cannot be empty!");

  // Load the graph.
  mRoadmap = boost::
      shared_ptr<io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>>(
          new io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>(
              mSpace, filename));

  mRoadmap->generate(
      mGraph,
      get(&VertexProperties::mState, mGraph),
      get(&EdgeProperties::mLength, mGraph));

  // Mark the graph to have been setup.
  mGraphSetup = true;
}

// ============================================================================
void GLS::setBestPathCost(double cost)
{
  mBestPathCost = cost;
}

// ============================================================================
double GLS::getBestPathCost()
{
  return mBestPathCost;
}

// ============================================================================
void GLS::setPlannerStatus(PlannerStatus status)
{
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus GLS::getPlannerStatus()
{
  return mPlannerStatus;
}

// ============================================================================
double GLS::getNumberOfEdgeEvaluations()
{
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double GLS::getNumberOfEdgeRewires()
{
  return mNumberOfEdgeRewires;
}

// ============================================================================
CollisionStatus GLS::evaluateVertex(Vertex v)
{
  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();
  
  auto state = mGraph[v].getState()->getOMPLState();

  // Evaluate the state.
  if (!validityChecker->isValid(state))
    return CollisionStatus::Collision;

  return CollisionStatus::Free;
}

// ============================================================================
CollisionStatus GLS::evaluateEdge(const Edge& e)
{
  mNumberOfEdgeEvaluations++;

  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  // Collision check the start and goal.
  Vertex startVertex = source(e, mGraph);
  Vertex endVertex = target(e, mGraph);

  auto startState = mGraph[startVertex].getState()->getOMPLState();
  auto endState = mGraph[endVertex].getState()->getOMPLState();

  // Evaluate Start and End States.
  if (!validityChecker->isValid(startState))
  {
    mGraph[startVertex].setCollisionStatus(CollisionStatus::Collision);
    return CollisionStatus::Collision;
  }

  if (!validityChecker->isValid(endState))
  {
    mGraph[endVertex].setCollisionStatus(CollisionStatus::Collision);
    return CollisionStatus::Collision;
  }

  // Evaluate the state in between.
  int maxSteps = 1.0 / mCollisionCheckResolution;
  for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier)
  {
    double interpolationStep = mCollisionCheckResolution * multiplier;
    assert(interpolationStep <= 1);
    StatePtr midVertex(new gls::datastructures::State(mSpace));
    mSpace->interpolate(
        startState, endState, interpolationStep, midVertex->getOMPLState());

    if (!validityChecker->isValid(midVertex->getOMPLState()))
      return CollisionStatus::Collision;
  }

  // Edge is collision-free.
  return CollisionStatus::Free;
}

// ============================================================================
void GLS::extendSearchTree()
{
  // Ideally extend search tree should not be called when the queue is empty.
  assert(!mExtendQueue.isEmpty());

  while (!mExtendQueue.isEmpty())
  {
    // Check if the popping the top vertex triggers the event.
    Vertex u = mExtendQueue.getTopVertex();
    if (mEvent->isTriggered(u))
    {
      std::cout << "Triggered with " << u << std::endl;
      Path p = getPathToSource(u);
      for (auto& vert : p)
      {
        std::cout << vert << " ";
      }
      std::cout << std::endl;
      // std::cin.get();
      break;
    }

    // Pop the top vertex in the queue to extend the search tree.
    u = mExtendQueue.popTopVertex();

    // The vertex being extended should have been marked visited.
    assert(mGraph[u].getVisitStatus() == VisitStatus::Visited);

    // If the vertex has been marked to be in collision, do not extend.
    if (mGraph[u].getCollisionStatus() == CollisionStatus::Collision)
      continue;

    // Get the neighbors and extend.
    // TODO (avk): Have a wrapper in case the implicit vs explicit.

    if (u == 81 || u == 41)
      std::cout << "Considering the kids of " << u << std::endl;    
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end;
         ++ni)
    {
      Vertex v = *ni;
      if (u == 81 || u == 41 || v == 185)
        std::cout << v << "considering parent " << u << ". ";

      // If the successor has been previously marked to be in collision,
      // continue to the next successor.
      if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
      {
        if (u == 81 || u == 41 || v == 185)
          std::cout << "in collision" << std::endl;
        continue;
      }

      // Enforce prevention of loops.
      if (v == mGraph[u].getParent())
      {
        if (u == 81 || u == 41 || v == 185)
          std::cout << "parent" << std::endl;
        continue;
      }

      // Never come back to the source.
      if (v == mSourceVertex)
        continue;

      // Get the edge between the two vertices.
      Edge uv = getEdge(u, v);

      // If the edge has been previously marked to be in collision,
      // continue to the next successor.
      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Collision)
      {
        if (u == 81 || u == 41 || v == 185)
          std::cout << "edge in collision" << std::endl;
        continue;
      }

      double edgeLength = mGraph[uv].getLength();
      if (mGraph[v].getVisitStatus() == VisitStatus::NotVisited)
      {
        if (u == 81 || u == 41 || v == 185)
          std::cout << "Not visited before. ";
        assert(v != mSourceVertex);
        mGraph[v].setVisitStatus(VisitStatus::Visited);
        assert(
            mExtendQueue.hasVertexWithValue(v, mGraph[v].getEstimatedTotalCost())
            == false);
      }
      else
      {
        double oldCostToCome = mGraph[v].getCostToCome();
        double newCostToCome = mGraph[u].getCostToCome() + edgeLength;

        // Use the parent ID to break ties.
        Vertex previousParent = mGraph[v].getParent();

        if (u == 81 || u == 41 || v == 185)
          std::cout << "Costs are " << oldCostToCome << "(" << previousParent << ")" << newCostToCome << ". ";

        // If the previous cost-to-come is lower, continue.
        if (oldCostToCome < newCostToCome)
          continue;

        // Tie-Breaking Rule
        if (oldCostToCome == newCostToCome)
        {
          if (previousParent < u)
            continue;
        }

        // The new cost is lower, make necessary updates.
        // Remove from previous siblings.
        mGraph[previousParent].removeChild(v);

        // Remove the previous version of the vertex from possible queues.
        if (mExtendQueue.hasVertexWithValue(v, mGraph[v].getEstimatedTotalCost()))
        {
          auto previousSize = mExtendQueue.getSize();
          mExtendQueue.removeVertexWithValue(v, mGraph[v].getEstimatedTotalCost());
          auto currentSize = mExtendQueue.getSize();
          assert(previousSize - currentSize == 1);
        }

        // Cascade the updates to all the descendents.
        // Replace this with getDescendents() function.
        std::vector<Vertex> subtree = {v};
        while (!subtree.empty())
        {
          auto iterT = subtree.rbegin();
          std::set<Vertex>& children = mGraph[*iterT].getChildren();
          subtree.pop_back();

          for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
          {
            mGraph[*iterS].setVisitStatus(VisitStatus::NotVisited);
            subtree.emplace_back(*iterS);
            if (mExtendQueue.hasVertexWithValue(*iterS, mGraph[*iterS].getEstimatedTotalCost()))
            {
              auto previousSize = mExtendQueue.getSize();
              mExtendQueue.removeVertexWithValue(
                  *iterS, mGraph[*iterS].getEstimatedTotalCost());
              auto currentSize = mExtendQueue.getSize();
              assert(previousSize - currentSize == 1);
            }
          }
          children.clear();
        }
      }

      // Update the successor's properties.
      mGraph[v].setParent(u);
      mGraph[v].setCostToCome(mGraph[u].getCostToCome() + edgeLength);
      mGraph[v].setHeuristic(getGraphHeuristic(v));

      if (u == 81 || u == 41 || v == 185)
        std::cout << "Parent: " << u << ". Cost To Come: " << mGraph[u].getCostToCome() + edgeLength << std::endl;

      // Update the vertex property associated with the event.
      mEvent->updateVertexProperties(v);

      // Add it to its new siblings
      mGraph[u].addChild(v);

      auto previousSize = mExtendQueue.getSize();
      mExtendQueue.addVertexWithValue(v, mGraph[v].getEstimatedTotalCost());
      auto currentSize = mExtendQueue.getSize();
      assert(currentSize - previousSize == 1);
    }
    if (u == 81 || u == 41)
    {
      std::cout << "Finished considering the kids of 81" << std::endl;
      mExtendQueue.printQueue(); 
    }
  }
}

// ============================================================================
void GLS::updateSearchTree()
{
  if (mTreeValidityStatus == TreeValidityStatus::Valid)
  {
    // Update the vertex properties of the entire search tree.
    mEvent->updateVertexProperties(mUpdateQueue);
    assert(mUpdateQueue.isEmpty());
  }
  else
  {
    // Rewire the search tree.
    rewireSearchTree();

    // With successful rewire, mark the tree to be valid again.
    mTreeValidityStatus = TreeValidityStatus::Valid;
  }
}

// ============================================================================
void GLS::rewireSearchTree()
{
  // 1. Collect all the vertices that need to be rewired.
  while (!mRewireQueue.isEmpty())
  {
    Vertex v = mRewireQueue.popTopVertex();
    if (v == 185)
      std::cout << "Going to rewire 185. Current parent is " << mGraph[v].getParent() << ". ";

    // Add all the children of the current node to mRewireQueue.
    auto children = mGraph[v].getChildren();
    for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
    {
      auto previousSize = mRewireQueue.getSize();
      mRewireQueue.addVertexWithValue(*iterS, mGraph[*iterS].getCostToCome());
      auto currentSize = mRewireQueue.getSize();
      assert(currentSize - previousSize == 1);
    }
    mGraph[v].removeAllChildren();

    // Add the vertex to set.
    mRewireSet.insert(v);

    // Remove from mExtendQueue.
    // TODO (avk): Can this happen?
    if (mExtendQueue.hasVertexWithValue(v, mGraph[v].getEstimatedTotalCost()))
    {
      auto previousSize = mExtendQueue.getSize();
      mExtendQueue.removeVertexWithValue(v, mGraph[v].getEstimatedTotalCost());
      auto currentSize = mExtendQueue.getSize();
      assert(previousSize - currentSize == 1);
    }

    // Assign default values
    mGraph[v].setParent(v);
    mGraph[v].setCostToCome(std::numeric_limits<double>::max());

    // Mark it as not visited
    mGraph[v].setVisitStatus(VisitStatus::NotVisited);
    mEvent->updateVertexProperties(v);
  }

  // 2. Assign the nodes keys
  assert(mRewireQueue.isEmpty());
  for (auto iterS = mRewireSet.begin(); iterS != mRewireSet.end(); ++iterS)
  {
    Vertex v = *iterS;

    // If the vertex has been marked to be in collision, ignore.
    if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
      continue;

    // Look for possible parents in the rest of the graph.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(v, mGraph); ni != ni_end;
         ++ni)
    {
      // Get the possible parent.
      Vertex u = *ni;

      if (v == 185)
        std::cout << "Considering rewiring to " << u << ". ";

      // If the parent has been marked in collision, ignore.
      if (mGraph[u].getCollisionStatus() == CollisionStatus::Collision)
      {
        if (v == 185)
          std::cout << "Parent in collision" << std::endl;
        continue;
      }

      // No point rewiring to the target vertex.
      if (u == mTargetVertex)
        continue;

      // If the neighbor is one of the vertices to be rewires, ignore now.
      if (mGraph[u].getCostToCome() == std::numeric_limits<double>::max())
      {
        if (v == 185)
          std::cout << "Cost to come to u is infinity" << std::endl;
        continue;
      }

      // If the neighbor is currently not in search tree, ignore.
      if (mGraph[u].getVisitStatus() == VisitStatus::NotVisited)
      {
        if (v == 185)
          std::cout << "Parent not visited" << std::endl;
        continue;
      }

      // If the parent is gonna trigger the event, ignore.
      if (mEvent->isTriggered(u))
      {
        if (v == 185)
          std::cout << "Parent in going to trigger" << std::endl;
        assert(mExtendQueue.hasVertexWithValue(u, mGraph[u].getEstimatedTotalCost()));
        continue;
      }

      // If the parent is already in mExtendQueue, can be rewired later.
      if (mExtendQueue.hasVertexWithValue(u, mGraph[u].getEstimatedTotalCost()))
      {
        if (v == 185)
        {
          std::cout << "Parent in extend" << std::endl;
        }
        continue;
      }

      assert(mRewireSet.find(u) == mRewireSet.end());

      Edge uv = getEdge(u, v);
      double edgeLength = mGraph[uv].getLength();

      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Free)
      {
        if (mGraph[v].getCostToCome() > mGraph[u].getCostToCome() + edgeLength
            || (mGraph[v].getCostToCome()
                    == mGraph[u].getCostToCome() + edgeLength
                && u < mGraph[v].getParent()))
        {
          // Update the vertex.
          mGraph[v].setCostToCome(mGraph[u].getCostToCome() + edgeLength);
          mGraph[v].setParent(u);

          if (v == 185)
            std::cout << "Rewiring temp to " << u << ". ";

          // Update the vertex property associated with the event.
          mEvent->updateVertexProperties(v);
        }
      }
    }
    auto previousSize = mRewireQueue.getSize();
    mRewireQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
    auto currentSize = mRewireQueue.getSize();
    assert(currentSize - previousSize == 1);
  }

  // 3. Start Rewiring in the cost space
  mRewireQueue.printQueue();
  while (!mRewireQueue.isEmpty())
  {
    Vertex u = mRewireQueue.popTopVertex();

    // Ignore loops.
    if (u == mGraph[u].getParent())
      continue;

    // Since valid parent is found, mark as visited.
    mGraph[u].setVisitStatus(VisitStatus::Visited);

    // Let the parent know of its new child.
    Vertex p = mGraph[u].getParent();
    mGraph[p].addChild(u);

    if (u == 185)
    {
      std::cout << "Rewiring permenantly to " << p << std::endl;
      mExtendQueue.printQueue();
    }

    auto previousSize = mExtendQueue.getSize();
    mExtendQueue.addVertexWithValue(u, mGraph[u].getEstimatedTotalCost());
    auto currentSize = mExtendQueue.getSize();
    assert(currentSize - previousSize == 1);

    // Check if u can be a better parent to the vertices being rewired.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end;
         ++ni)
    {
      Vertex v = *ni;

      if (v == 185)
      {
        std::cout << "3rd step: Considering possible parent to rewire to " << u << std::endl;
      }

      // TODO (avk): Is this necessary?
      if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // Vertex needs to be in set to update.
      if (mRewireSet.find(v) == mRewireSet.end())
      {
        if (v == 185)
        {
          std::cout << "3rd step: Continuing since 185 is not in rewire set" << std::endl;
        }        
        continue;
      }

      Edge uv = getEdge(u, v);
      double edgeLength = mGraph[uv].getLength();

      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Free)
      {
        if (mGraph[v].getCostToCome() > mGraph[u].getCostToCome() + edgeLength
            || (mGraph[v].getCostToCome()
                    == mGraph[u].getCostToCome() + edgeLength
                && u < mGraph[v].getParent()))
        {

          if (v == 185)
            std::cout << "Looking like " << u << " with " << mGraph[u].getCostToCome() + edgeLength << " is better than " << mGraph[v].getParent() << " with " << mGraph[v].getCostToCome() << std::endl;

          if (mRewireQueue.hasVertexWithValue(v, mGraph[v].getCostToCome()))
          {
            auto previousSize = mRewireQueue.getSize();
            mRewireQueue.removeVertexWithValue(v, mGraph[v].getCostToCome());
            auto currentSize = mRewireQueue.getSize();
            assert(previousSize - currentSize == 1);
          }

          assert(mExtendQueue.hasVertexWithValue(u, mGraph[u].getEstimatedTotalCost()));
          // if (mExtendQueue.hasVertexWithValue(u, mGraph[u].getEstimatedTotalCost()))
          // {
            // if (v == 185)
              // std::cout << "Since " << u << "is in the extend queue, we will remove rewiring of " << v << std::endl;
            // mGraph[v].setVisitStatus(VisitStatus::NotVisited);
            // mGraph[v].setCostToCome(std::numeric_limits<double>::max());
            // mGraph[v].setParent(v);
            // mEvent->updateVertexProperties(v);
            // continue;
          // }

          // Update the vertex.
          mGraph[v].setCostToCome(mGraph[u].getCostToCome() + edgeLength);
          mGraph[v].setParent(u);

          // Update the vertex property associated with the event.
          mEvent->updateVertexProperties(v);
          
          auto previousSize = mRewireQueue.getSize();
          mRewireQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
          auto currentSize = mRewireQueue.getSize();
          assert(currentSize - previousSize == 1);
        }
      }
    }
  }
  mRewireSet.clear();
  assert(mRewireQueue.isEmpty());
}

// ============================================================================
void GLS::evaluateSearchTree()
{
  if (mExtendQueue.isEmpty())
    return;

  Vertex bestVertex = mExtendQueue.getTopVertex();
  Path edgeToEvaluate
      = mSelector->selectEdgeToEvaluate(getPathToSource(bestVertex));

  // Only one edge should have been selected for evaluation.
  assert(edgeToEvaluate.size() == 2);

  Vertex u = edgeToEvaluate[0];
  Vertex v = edgeToEvaluate[1];
  Edge uv = getEdge(u, v);

  // Assume that the selector might return edges already evaluated.
  assert(mGraph[uv].getEvaluationStatus() != EvaluationStatus::Evaluated);

  // Evaluate the edge.
  mGraph[uv].setEvaluationStatus(EvaluationStatus::Evaluated);
  if (evaluateEdge(uv) == CollisionStatus::Free)
  {
    // Set the edge collision status.
    mGraph[uv].setCollisionStatus(CollisionStatus::Free);

    // Populate the queue to update the search tree.
    auto previousSize = mUpdateQueue.getSize();
    mUpdateQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
    auto currentSize = mUpdateQueue.getSize();
    assert(currentSize - previousSize == 1);
  }
  else
  {
    mGraph[uv].setCollisionStatus(CollisionStatus::Collision);
    mTreeValidityStatus = TreeValidityStatus::NotValid;

    // Let the old parent know that the child has been removed.
    Vertex previousParent = mGraph[v].getParent();
    mGraph[previousParent].removeChild(v);
   
    // Add to the rewire queue.
    auto previousSize = mRewireQueue.getSize();
    mRewireQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
    auto currentSize = mRewireQueue.getSize();
    assert(currentSize - previousSize == 1);
  }

  if (bestVertex == mTargetVertex && mTreeValidityStatus == TreeValidityStatus::Valid)
  {
    if (foundPathToGoal())
    {
      // Planning problem has been solved!
      mPlannerStatus = PlannerStatus::Solved;
      auto c185 = mGraph[185].getChildren();
      auto c77 = mGraph[77].getChildren();
      for (auto& c : c185)
        std::cout << c << " ";
      std::cout << std::endl;
      for (auto& c : c77)
        std::cout << c << " ";
      std::cout << std::endl;

      std::cout << "185 " << mGraph[185].getParent() << " " << mGraph[185].getVisitStatus() << " " << mGraph[185].getCostToCome() << " " << mGraph[81].getCostToCome() << std::endl;
    }
  }

  // Based on the evaluation, update the search tree.
  updateSearchTree();
}

// ============================================================================
ompl::base::PathPtr GLS::constructSolution(
    const Vertex& source, const Vertex& target)
{
  ompl::geometric::PathGeometric* path
      = new ompl::geometric::PathGeometric(si_);
  Vertex v = target;

  while (v != source)
  {
    path->append(mGraph[v].getState()->getOMPLState());
    v = mGraph[v].getParent();
  }

  if (v == source)
  {
    path->append(mGraph[source].getState()->getOMPLState());
  }
  path->reverse();
  return ompl::base::PathPtr(path);
}

} // namespace gls
