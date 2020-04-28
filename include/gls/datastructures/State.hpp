/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_STATE_HPP_
#define GLS_DATASTRUCTURES_STATE_HPP_

#include <ompl/base/StateSpace.h>

#include "gls/GLS.hpp"

namespace gls {

class GLS::State {
 public:
  /// Constructor.
  State();

  /// Constructor.
  /// \param[in] space Statespace the state belongs to.
  explicit State(ompl::base::StateSpacePtr space);

  /// Destructor.
  ~State();

  /// Get OMPL state.
  // TODO (avk): Should I make this function const?
  ompl::base::State* getOMPLState();

  /// Get OMPL space.
  // TODO (avk): Should I make this function const?
  ompl::base::StateSpacePtr getStateSpace();

 private:
  /// The OMPL statespace operating on.
  const ompl::base::StateSpacePtr mSpace;

  /// The OMPL state that is being wrapped.
  ompl::base::State* mState;
};

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_STATE_HPP_
