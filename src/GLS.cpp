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



} // namespace gls