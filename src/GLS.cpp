/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/GLS.hpp"

#include <algorithm> // std::reverse
#include <cmath>     // pow, sqrt
#include <iostream>  // std::invalid_argument
#include <set>       // std::set
#include <assert.h>  // debug

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
  mConnectionRadius = 0.0;
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

  // TODO (avk): If the graph is not provided, use implicit representation
  // for the edges using the NearestNeighbor representation.
  // Check if roadmap has been provided.

  if (mRoadmapFilename == "")
    std::invalid_argument("Roadmap Filename cannot be empty!");

  mRoadmap = boost::
      shared_ptr<io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>>(
          new io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>(
              mSpace, mRoadmapFilename));

  mRoadmap->generate(
      mGraph,
      get(&VertexProperties::mState, mGraph),
      get(&EdgeProperties::mLength, mGraph));

  // Set default vertex values.
  // TODO (avk): Why should this be done a-priori? Test that these are set by default.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
  {
    mGraph[*vi].setCostToCome(std::numeric_limits<double>::infinity());
    mGraph[*vi].setHeuristic(0);
    mGraph[*vi].setVisitStatus(VisitStatus::NotVisited);
    mGraph[*vi].setCollisionStatus(CollisionStatus::Free);
  }

  // Set default edge values.
  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei)
  {
    mGraph[*ei].setEvaluationStatus(EvaluationStatus::NotEvaluated);
    mGraph[*ei].setCollisionStatus(CollisionStatus::Free);
  }
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
}

// ============================================================================
void GLS::setupPreliminaries()
{
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
  mGraph[mSourceVertex].setParent(-1);

  mGraph[mTargetVertex].setCostToCome(std::numeric_limits<double>::infinity());
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
    }
  }

  // Setup the event.
  mEvent->setup(mGraph, mSourceVertex, mTargetVertex);

  // Setup the selector.
  mSelector->setup(mGraph, mSourceVertex, mTargetVertex);
}

// ============================================================================
void GLS::clear()
{
  // TODO (avk): implement resetVertexProperties() and resetEdgeProperties()
}

// ============================================================================
ompl::base::PlannerStatus GLS::solve(
    const ompl::base::PlannerTerminationCondition& /*ptc*/)
{
  // TODO (avk): Use ptc to terminate the search.

  // Add the source vertex to the search tree with zero cost-to-come.
  mGraph[mSourceVertex].setVisitStatus(VisitStatus::Visited);
  mExtendQueue.addVertexWithValue(mSourceVertex, 0);

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

  return ompl::base::PlannerStatus::TIMEOUT;
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
  return pathToSource;
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
void GLS::setRoadmapFilename(std::string filename)
{
  mRoadmapFilename = filename;
}

// ============================================================================
std::string GLS::getRoadmapFilename()
{
  return mRoadmapFilename;
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

// ===========================================================================================
CollisionStatus GLS::evaluateEdge(const Edge&)
{
  auto validityChecker = si_->getStateValidityChecker();

  return CollisionStatus::Free;
}

// ============================================================================
void GLS::extendSearchTree()
{
  while (!mExtendQueue.isEmpty())
  {
    // Check if the popping the top vertex triggers the event.
    Vertex u = mExtendQueue.getTopVertex();
    if (mEvent->isTriggered(u))
      break;

    // Pop the top vertex in the queue to extend the search tree.
    u = mExtendQueue.popTopVertex();

    // The vertex being extended should have been marked visited.
    assert(mGraph[u].getVisitStatus() == VisitStatus::Visited);

    // If the vertex has been marked to be in collision, do not extend.
    if (mGraph[u].getCollisionStatus() == CollisionStatus::Collision)
      continue;

    // Get the neighbors and extend.
    // TODO (avk): Have a wrapper in case the implicit vs explicit.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end;
         ++ni)
    {
      // Get the successor vertex.
      Vertex v = *ni;

      // If the successor has been previously marked to be in collision,
      // continue to the next successor.
      if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // Enforce prevention of loops.
      if (v == mGraph[u].getParent())
        continue;

      // Get the edge between the two vertices.
      Edge uv = getEdge(u, v);

      // If the edge has been previously marked to be in collision,
      // continue to the next successor.
      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      double edgeLength = mGraph[uv].getLength();
      if (mGraph[v].getVisitStatus() == VisitStatus::NotVisited)
      {
        mGraph[v].setVisitStatus(VisitStatus::Visited);
        assert(
            mExtendQueue.hasVertexWithValue(v, mGraph[v].getCostToCome())
            == false);
      }
      else
      {
        double oldCostToCome = mGraph[v].getCostToCome();
        double newCostToCome = mGraph[u].getCostToCome() + edgeLength;

        // Use the parent ID to break ties.
        Vertex previousParent = mGraph[v].getParent();

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
        mExtendQueue.removeVertexWithValue(v, oldCostToCome);

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
            mExtendQueue.removeVertexWithValue(
                *iterS, mGraph[*iterS].getCostToCome());
          }
          children.clear();
        }
      }

      // Update the successor's properties.
      mGraph[v].setParent(u);
      mGraph[v].setCostToCome(mGraph[u].getCostToCome() + edgeLength);
      mGraph[v].setHeuristic(getGraphHeuristic(v));
      mEvent->updateVertexProperties(v);

      // Add it to its new siblings
      mGraph[u].addChild(v);
      mExtendQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
    }
  }
}

// ============================================================================
void GLS::updateSearchTree()
{
  // Do nothing
  // If rewiring, make sure to change the mTreeValidityStatus back.
  if (mTreeValidityStatus == TreeValidityStatus::Valid)
    return;
  // mEvent->updateVertexProperties(mUpdateQueue);
  else
    rewireSearchTree();
}

// ============================================================================
void GLS::rewireSearchTree()
{
  // 1. Collect all the vertices that need to be rewired.
  while (!mRewireQueue.isEmpty())
  {
    Vertex v = mRewireQueue.popTopVertex();

    // Add all the children of the current node to mRewireQueue.
    auto children = mGraph[v].getChildren();
    for (auto iterS = children.begin(); iterS != children.end(); ++iterS)
    {
      mRewireQueue.addVertexWithValue(*iterS, mGraph[*iterS].getCostToCome());
    }
    mGraph[v].removeAllChildren();

    // Add the vertex to set.
    mRewireSet.insert(v);

    // Remove from mExtendQueue.
    // TODO (avk): Can this happen?
    mExtendQueue.removeVertexWithValue(v, mGraph[v].getCostToCome());

    // Assign default values
    mGraph[v].setParent(v);
    mGraph[v].setCostToCome(std::numeric_limits<double>::max());

    // Mark it as not visited
    mGraph[v].setVisitStatus(VisitStatus::NotVisited);
  }

  // 2. Assign the nodes keys
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

      // If the parent has been marked in collision, ignore.
      if (mGraph[u].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // No point rewiring to the target vertex.
      if (u == mTargetVertex)
        continue;

      // If the neighbor is one of the vertices to be rewires, ignore now.
      if (mGraph[u].getCostToCome() == std::numeric_limits<double>::max())
        continue;

      // If the neighbor is currently not in search tree, ignore.
      if (mGraph[u].getVisitStatus() == VisitStatus::NotVisited)
        continue;

      // If the parent is gonna trigger the event, ignore.
      if (mEvent->isTriggered(u))
        continue;

      // If the parent is already in mExtendQueue, can be rewired later.
      if (mExtendQueue.hasVertexWithValue(u, mGraph[u].getCostToCome()))
        continue;

      assert(mRewireSet.find(u) == mRewireSet.end());

      Edge uv = getEdge(u, v);
      double edgeLength = mGraph[uv].getLength();

      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Free)
      {
        if (mGraph[v].getCostToCome() > mGraph[u].getCostToCome() + edgeLength
            || (mGraph[v].getCostToCome()
                    > mGraph[u].getCostToCome() + edgeLength
                && u < mGraph[v].getParent()))
        {
          mGraph[v].setCostToCome(mGraph[u].getCostToCome() + edgeLength);
          mGraph[v].setParent(u);
          mEvent->updateVertexProperties(v); // no need to cascade.
        }
      }
    }
    mRewireQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
  }

  // 3. Start Rewiring in the cost space
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

    // Add u for further extension.
    if (!mEvent->isTriggered(u))
    {
      mExtendQueue.addVertexWithValue(u, mGraph[u].getCostToCome());
    }

    // Check if u can be a better parent to the vertices being rewired.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end;
         ++ni)
    {
      Vertex v = *ni;

      // TODO (avk): Is this necessary?
      if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // Vertex needs to be in set to update.
      if (mRewireSet.find(v) == mRewireSet.end())
        continue;

      Edge uv = getEdge(u, v);
      double edgeLength = mGraph[uv].getLength();

      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Free)
      {
        if (mGraph[v].getCostToCome() > mGraph[u].getCostToCome() + edgeLength
            || (mGraph[v].getCostToCome()
                    == mGraph[u].getCostToCome() + edgeLength
                && u < mGraph[v].getParent()))
        {
          mRewireQueue.removeVertexWithValue(v, mGraph[v].getCostToCome());
          if (mExtendQueue.hasVertexWithValue(u, mGraph[u].getCostToCome()))
          {
            mGraph[v].setVisitStatus(VisitStatus::NotVisited);
            mGraph[v].setCostToCome(std::numeric_limits<double>::max());
            mGraph[v].setParent(v);
            continue;
          }

          mGraph[v].setCostToCome(mGraph[u].getCostToCome() + edgeLength);
          mGraph[v].setParent(u);
          mEvent->updateVertexProperties(v); // no need to cascade.
          mRewireQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
        }
      }
    }
  }
  mRewireSet.clear();
}

// ============================================================================
void GLS::evaluateSearchTree()
{
  Vertex u = mExtendQueue.getTopVertex();
  Path edgesToEvaluate = mSelector->selectEdgesToEvaluate(getPathToSource(u));

  for (std::size_t i = 0; i < edgesToEvaluate.size() - 1; ++i)
  {
    Vertex u = edgesToEvaluate[i];
    Vertex v = edgesToEvaluate[i + 1];
    Edge uv = getEdge(u, v);
    mGraph[uv].setEvaluationStatus(EvaluationStatus::Evaluated);
    if (evaluateEdge(uv) == CollisionStatus::Free)
    {
      mGraph[uv].setCollisionStatus(CollisionStatus::Free);
      mUpdateQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
    }
    else
    {
      mGraph[uv].setCollisionStatus(CollisionStatus::Collision);
      mTreeValidityStatus = TreeValidityStatus::NotValid;
      mRewireQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
      return;
    }
  }

  if (u == mTargetVertex)
  {
    mPlannerStatus = PlannerStatus::Solved;
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
