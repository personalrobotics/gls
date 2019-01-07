/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/State.hpp"

namespace gls {
namespace datastructures {

// ================================================================================================
State::State()
{
  // Do nothing.
}

// ================================================================================================
State::State(ompl::base::StateSpacePtr space)
  : mSpace(space), mState(space->allocState())
{
  // Do nothing.
}

// ================================================================================================
State::~State()
{
  mSpace->freeState(this->mState);
}

// ================================================================================================
ompl::base::State* State::getOMPLState()
{
  return mState;
}

// ================================================================================================
ompl::base::StateSpacePtr State::getStateSpace()
{
  return mSpace;
}

} // namespace datastructures
} // namespace gls
