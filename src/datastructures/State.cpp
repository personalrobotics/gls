/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/State.hpp"

namespace gls {

// ================================================================================================
GLS::State::State() {
  // Do nothing.
}

// ================================================================================================
GLS::State::State(ompl::base::StateSpacePtr space)
    : mSpace(space), mState(space->allocState()) {
  // Do nothing.
}

// ================================================================================================
GLS::State::~State() { mSpace->freeState(this->mState); }

// ================================================================================================
ompl::base::State* GLS::State::getOMPLState() { return mState; }

// ================================================================================================
ompl::base::StateSpacePtr GLS::State::getStateSpace() { return mSpace; }

}  // namespace gls
