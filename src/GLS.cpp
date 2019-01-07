/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/GLS.hpp"

#include <algorithm> // std::reverse
#include <cmath>     // pow, sqrt
#include <set>       // std::set
#include <assert.h>  // debug

using gls::datastructures::CollisionStatus;
using gls::datastructures::Edge;
using gls::datastructures::EdgeIter;
using gls::datastructures::EdgeProperties;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::NeighborIter;
using gls::datastructures::StatePtr;
using gls::datastructures::Vertex;
using gls::datastructures::VertexIter;
using gls::datastructures::VertexProperties;
using gls::datastructures::VisitStatus;

namespace gls {

GLS::GLS(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "GLS"), mSpace(si->getStateSpace())
{
  // Do Nothing.
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

  mRoadmap->generate(mGraph, get(&VertexProperties::mState, mGraph),
                          get(&EdgeProperties::mLength, mGraph));

  // Set default vertex values.
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

  addSourceAndTargetToGraph();
}

void GLS::addSourceAndTargetToGraph()
{
  // TODO (avk): need mConnectionRadius
  double mConnectionRadius = 0;

  StatePtr sourceState(new gls::datastructures::State(mSpace));
  mSpace->copyState(sourceState->getOMPLState(), pdef_->getStartState(0));

  StatePtr targetState(new gls::datastructures::State(mSpace));
  mSpace->copyState(targetState->getOMPLState(), pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

  // Add start and goal vertices to the graph
  // TODO (avk): Implement setState();
  mSourceVertex = boost::add_vertex(mGraph);
  // mGraph[mSourceVertex].setState(sourceState);

  mTargetVertex = boost::add_vertex(mGraph);
  // mGraph[mTargetVertex].setState(targetState);

  // Assign default values. // TODO (avk): Implement the heurstic function.
  mGraph[mSourceVertex].setCostToCome(0);
  mGraph[mSourceVertex].setHeuristic(0);
  mGraph[mSourceVertex].setVisitStatus(VisitStatus::NotVisited);
  mGraph[mSourceVertex].setCollisionStatus(CollisionStatus::Free);
  mGraph[mSourceVertex].setParent(-1);

  mGraph[mTargetVertex].setCostToCome(std::numeric_limits<double>::infinity());
  mGraph[mTargetVertex].setHeuristic(0);
  mGraph[mTargetVertex].setVisitStatus(VisitStatus::NotVisited);
  mGraph[mTargetVertex].setCollisionStatus(CollisionStatus::Free);

  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi)
  {
    double sourceDistance = mSpace->distance(mGraph[*vi].getState()->getOMPLState(), sourceState->getOMPLState());
    double targetDistance = mSpace->distance(mGraph[*vi].getState()->getOMPLState(), targetState->getOMPLState());

    if (sourceDistance < mConnectionRadius)
    {
      if(mSourceVertex == *vi)
        continue;

      std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, *vi, mGraph);
      
      mGraph[newEdge.first].setLength(sourceDistance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);
    }

    if (targetDistance < mConnectionRadius)
    {
      if(mTargetVertex == *vi)
        continue;

      std::pair<Edge,bool> newEdge = boost::add_edge(mTargetVertex, *vi, mGraph);
      mGraph[newEdge.first].setLength(targetDistance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);
    }
  }
}

// ============================================================================
void GLS::clear()
{
  // TODO (avk): implement resetVertexProperties() and resetEdgeProperties()
}

// ============================================================================
ompl::base::PlannerStatus GLS::solve(
    const ompl::base::PlannerTerminationCondition& ptc)
{
  // TODO (avk): Use ptc to terminate the search.

  bool solutionFound = false;

  // Add the source vertex to the search tree with zero cost-to-come.
  mGraph[mSourceVertex].setVisitStatus(VisitStatus::Visited);
  mExtendQueue.addVertexWithValue(mSourceVertex, 0);

  // Run in loop.
  while (!solutionFound || !mExtendQueue.isEmpty())
  {
    // Extend the tree till the event is triggered.
    extendSearchTree();

    // Evaluate the tree.
    // TODO (avk): Think of the API here.
    // if (evaluateSearchTree() == TreeValidityStatus::NotValid)
    //   rewireSearchTree();
    // else
    //   updateSearchTree()
  }

  // TODO (avk): Replace this with !solutionFound PlannerStatus::Failure
  // if it exists.
  if (solutionFound)
  {
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
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
      mGraph[v].setCostToCome(mGraph[u].getCostToCome());
      mGraph[v].setHeuristic(0); // TODO (avk): heuristicFunction(v) in graph.
      // TODO (avk): We need to update the event-dependent attributes.

      // Add it to its new siblings
      mGraph[u].addChild(v);
      mExtendQueue.addVertexWithValue(v, mGraph[v].getCostToCome());
    }
  }
}

// ============================================================================
void GLS::rewireSearchTree()
{
  // Do nothing
}

// ============================================================================
TreeValidityStatus GLS::evaluateSearchTree()
{
  // Do nothing
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
    // TODO (avk): Involve the state wrapper.
    // path->append(graph[v].state->state);
    v = mGraph[v].getParent();
  }

  if (v == source)
  {
    // path->append(graph[start].state->state);
  }
  path->reverse();
  return ompl::base::PathPtr(path);
}

} // namespace gls