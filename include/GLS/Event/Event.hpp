/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_EVENT_EVENT_HPP_
#define GLS_EVENT_EVENT_HPP_

#include <string>   // std::string
#include <utility>  // std::pair
#include <vector>   // std::vector

namespace gls {
namespace event {

/// Event is a base class to define the trigger to pause search.
/// The rule for switching between serach and edge evaluation is
/// specified by the concrete classes.
class Event
{
public:
  /// Constructor.
  Event();

  /// Destructor.
  virtual ~Event() = default;

  // Checks if the event is triggered.
  virtual bool isTriggered(std::size_t vertex) = 0;

}; // Event

} // event
} // gls

#endif // GLS_EVENT_EVENT_HPP_
