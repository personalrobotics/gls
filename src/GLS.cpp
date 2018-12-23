/* Authors: Aditya Vamsikrishna Mandalika */

#include "GLS/GLS.hpp"

#include <algorithm> // std::reverse
#include <cmath>     // pow, sqrt
#include <set>       // std::set
#include <assert.h>  // debug

using gls::datastructures::CollisionStatus;
using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::NeighborIter;
using gls::datastructures::Vertex;
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

  // TODO (avk): Time to pull in state wrapper around OMPL state.
  // TODO (avk): Set the start and goal states.
  // TODO (avk): implement addStartAndGoalToGraph().
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
  // Do nothing
}

// ============================================================================
double GLS::getTopFrontierQueueValue()
{
  double costReference;
  if(mFrontierQueue.isEmpty())
  {
    costReference = std::numeric_limits<double>::max();
  }
  else
  {
    costReference = mFrontierQueue.getTopVertexValue();
  }
  return costReference;
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

    // Extend the search tree further only if cost of the vertex to extend is
    // less than the cost best vertex in the frontier.
    double costReference = getTopFrontierQueueValue();
    if(mGraph[u].getEstimatedTotalCost() >= costReference)
      break;

    // The vertex being extended should have been marked visited.
    assert(mGraph[u].getVisitStatus() == VisitStatus::Visited);

    // If the vertex popped is goal, add it to the frontier queue.
    if(u == mGoalVertex)
      mFrontierQueue.addVertexWithValue(u, mGraph[u].getEstimatedTotalCost());

    // If the vertex has been marked to be in collision, do not extend.
    if(mGraph[u].getCollisionStatus() == CollisionStatus::Collision)
      continue;
    
    // Get the neighbors and extend.
    // TODO (avk): Have a wrapper in case the implicit vs explicit.
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end; ++ni)
    {
      // Get the successor vertex.
      Vertex v = *ni;

      // If the successor has been previously marked to be in collision, continue.
      if (mGraph[v].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // Enforce prevention of loops.
      if (v == mGraph[u].getParent())
        continue;

      // Get the edge between the two vertices.
      Edge uv = getEdge(u, v);

      // If the edge has been previously marked to be in collision, continue.
      if (mGraph[uv].getCollisionStatus() == CollisionStatus::Collision)
        continue;

      // double edgeLength = mGraph[uv].getLength();

      if (mGraph[v].getVisitStatus() == VisitStatus::NotVisited)
      {
        mGraph[v].setVisitStatus(VisitStatus::Visited);
        // assert(mExtendQueue.hasVertex(v) == false); // TODO (avk): hasVertex()
        // assert(mFrontierQueue.hasVertex(v) == false);
      }
      else
      {
        // TODO (avk): Check if the new cost is better in which case, update.
      }

      // Update the successor's properties.
      mGraph[v].setParent(u);
      mGraph[v].setCostToCome(mGraph[u].getCostToCome());
      mGraph[v].setHeuristic(0); // TODO (avk): heuristicFunction(v) in graph.
      
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
void GLS::evaluateSearchTree()
{
  // Do nothing
}

} // namespace gls