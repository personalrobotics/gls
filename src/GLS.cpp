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
  // Do nothing
}

// ============================================================================
void GLS::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
  // Do nothing
}

// ============================================================================
void GLS::clear()
{
  // Do nothing
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