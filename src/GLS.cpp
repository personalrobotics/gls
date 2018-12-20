/* Authors: Aditya Vamsikrishna Mandalika */

#include "GLS/GLS.hpp"

#include <algorithm>        // std::reverse
#include <cmath>            // pow, sqrt
#include <set>              // std::set
#include <assert.h>         // debug

namespace gls {

GLS::GLS(const ompl::base::SpaceInformationPtr &si)
  : ompl::base::Planner(si, "GLS")
  , mSpace(si->getStateSpace())
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
}

// ============================================================================
void GLS::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
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
  const ompl::base::PlannerTerminationCondition &ptc)
{
  // Do nothing
}

// ============================================================================
void GLS::extendSearchTree()
{
  // Do nothing
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