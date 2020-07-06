// Author: Aditya Vamsikrishna Mandalika
// Date: 5th July 2020

#ifndef GLS_EVENT_EVENT_HPP_
#define GLS_EVENT_EVENT_HPP_

#include <vector>

#include "gls/datastructures/Graph.hpp"

namespace gls {

/// \brief Base class for an event that determines when search pauses.
class Event {
 public:
  /// Constructor.
  /// \param[in] graph The graph for any auxiliary information.
  Event(const Graph& graph, const std::size_t sourceIndex,
        const std::size_t targetIndex, const std::string& name = "Event")
      : mGraph(graph),
        mSourceIndex(sourceIndex),
        mTargetIndex(targetIndex),
        mName(name) {
    // Do nothing.
  }

  /// Destructor.
  virtual ~Event() = default;

  /// The core function of the Event. Returns a boolean if it is triggered.
  /// By default, it returns true if the \c index is equal to goal index.
  virtual bool inline isTriggered(const std::size_t& index) const {
    return index == mTargetIndex;
  };

  // Returns the name associated with the event.
  const inline std::string& getName() const { return mName; }

 protected:
  // The associated graph.
  const Graph& mGraph;

  // The source index.
  const std::size_t& mSourceIndex;

  // The target index.
  const std::size_t& mTargetIndex;

  // The name of the event.
  const std::string& mName;
};

}  // namespace gls

#endif  // GLS_EVENT_EVENT_HPP_
