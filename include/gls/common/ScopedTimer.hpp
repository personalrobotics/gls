/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_COMMON_SCOPEDTIMER_HPP_
#define GLS_COMMON_SCOPEDTIMER_HPP_

#include <chrono>         // std::chrono
#include <string>         // std::string
#include <unordered_map>  // std::unordered_map
#include <vector>         // std::vector

namespace gls {

/// A class that clocks the time spent in a particular scope. The clock starts
/// when the object is constructed and stops when the object goes out of scope.
class ScopedTimer {
 public:
  /// \brief Constructor.
  /// Starts the clock. Name of the timer is its ID. If a timer with ID "name"
  /// already exists, the time spent is incremented over previous time.
  ScopedTimer(const std::string name);

  /// \brief Destructor.
  /// Stops the clock.
  ~ScopedTimer();

  /// \brief Returns the name of the timer.
  std::string getName() const;

  /// \brief Returns the number of times the timer was called.
  std::size_t getCount() const;

  /// \brief Returns the time spent on this scoped timer until now.
  double getCurrentTime() const;

  /// \brief Returns the total time spent on this scoped timer.
  double getTotalTime() const;

  /// \brief Returns the name of the parent scoped timer, if parent exists.
  std::string getParentName() const;

  /// \brief Returns the nested level, starts at 0.
  std::size_t getNestedLevel() const;

  /// \brief Returns the map of timer IDs and the objects.
  static const std::unordered_map<std::string, ScopedTimer>
  getScopedTimersMap();

 private:
  /// Name of the associated timer.
  std::string mName;
  /// Number of times the scoped timer was invoked.
  std::size_t mCount{0};
  /// Total time spent in the scope.
  double mTotalTime{0.0};
  /// Parent scope name.
  std::string mParent;
  /// Nested level.
  std::size_t mNestedLevel{0};
  /// Start time of the timer.
  std::chrono::time_point<std::chrono::system_clock> mStartTime;
  /// \brief Static map from the timer IDs to the objects.
  static std::unordered_map<std::string, ScopedTimer> mScopedTimersMap;
  /// \brief Static vector of active scoped timers.
  static std::vector<ScopedTimer> mActiveScopedTimers;
};

}  // namespace gls

#endif  // GLS_COMMON_SCOPEDTIMER_HPP_
