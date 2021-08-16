/// Author: Aditya Mandalika.
/// Date: 7th October 2020.

#ifndef GLS_COMMON_SCOPEDTIMER_HPP_
#define GLS_COMMON_SCOPEDTIMER_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gls {
namespace common {

///
/// \brief Defines the assets of a scoped timer.
///
struct ScopedTimerAssets {
  // Time spent running the associated scoped timer.
  double mTotalTime{0};

  // Name of the parent timer.
  std::string mParentTimerName{""};

  // Number of times the associated timer was invoked.
  std::size_t mCount{0};

  // Nested level of the associated timer.
  std::size_t mNestedLevel{0};
};

class SimpleScopedTimer {
public:
  ///
  /// \brief Construct a new Scoped Timer object. The timer is ID'd by its name.
  ///
  /// \param name The name of the timer.
  explicit SimpleScopedTimer(std::string name, std::string unit)
    : mName(name), mUnit(unit), mStartTime(std::chrono::system_clock::now()) {
  }

  ///
  /// \brief Stops the clock when the timer goes out of scope.
  ///
  ~SimpleScopedTimer() {
    std::chrono::time_point<std::chrono::system_clock> endTime(std::chrono::system_clock::now());

    double elapsedTime;
    if (mUnit == "milliseconds") {
      elapsedTime = std::chrono::duration<double, std::milli>(endTime - mStartTime).count();
    } else if (mUnit == "microseconds") {
      elapsedTime = std::chrono::duration<double, std::micro>(endTime - mStartTime).count();
    } else if (mUnit == "nanoseconds") {
      elapsedTime = std::chrono::duration<double, std::nano>(endTime - mStartTime).count();
    } else {
      elapsedTime = std::chrono::duration<double>(endTime - mStartTime).count();
    }
    std::cout << mName << ": " << elapsedTime << std::endl;
  }

private:
  /// Name of the associated timer.
  const std::string mName;

  /// Units to cast into.
  const std::string mUnit;

  /// Start time of the timer.
  std::chrono::time_point<std::chrono::system_clock> mStartTime;
};

///
/// \brief A class that clocks the time spent in a particular scope. The clock
/// starts when the object is constructed and stops when the object goes out of
/// scope.
///
class ScopedTimer {
public:
  ///
  /// \brief Construct a new Scoped Timer object. The timer is ID'd by its name.
  ///
  /// \param name The name of the timer.
  ScopedTimer(const std::string& name) : mName(name) {
    if (!mActiveScopedTimers.empty()) {
      mAssets.mParentTimerName = mActiveScopedTimers.back();
      mAssets.mNestedLevel = mActiveScopedTimers.size();
    }
    mAssets.mCount += 1;
    mActiveScopedTimers.emplace_back(name);
    mStartTime = std::chrono::system_clock::now();
  }

  ///
  /// \brief Stops the clock when the timer goes out of scope.
  ///
  ~ScopedTimer() {
    std::chrono::time_point<std::chrono::system_clock> endTime(std::chrono::system_clock::now());
    std::chrono::duration<double> totalTime{endTime - mStartTime};

    auto iter = mScopedTimerAssetsMap.find(mName);
    if (iter == mScopedTimerAssetsMap.end()) {
      mAssets.mTotalTime = totalTime.count();
      mScopedTimerAssetsMap[mName] = mAssets;
    } else {
      iter->second.mTotalTime += totalTime.count();
    }
  }

  /// \brief Returns the name of the timer.
  std::string getName() const {
    return mName;
  }

  /// \brief Returns the time spent on the current timer.
  double getCurrentTime() const {
    std::chrono::time_point<std::chrono::system_clock> currentTime(
        std::chrono::system_clock::now());
    std::chrono::duration<double> totalTime{currentTime - mStartTime};
    return totalTime.count();
  }

  /// \brief Returns the total time spent on this scoped timer including past
  /// runs on timers with same ID.
  double getTotalTime() const {
    auto iter = mScopedTimerAssetsMap.find(mName);
    if (iter == mScopedTimerAssetsMap.end()) {
      return getCurrentTime();
    }
    return getCurrentTime() + iter->second.mTotalTime;
  }

  /// \brief Returns the map of timer IDs and the objects.
  static const std::unordered_map<std::string, ScopedTimerAssets> getScopedTimerAssetsMap() {
    return mScopedTimerAssetsMap;
  }

private:
  /// Name of the associated timer.
  const std::string& mName;

  /// Start time of the timer.
  std::chrono::time_point<std::chrono::system_clock> mStartTime;

  /// Assets associated with this timer.
  ScopedTimerAssets mAssets;

  /// \brief Static map from the timer IDs to the objects.
  inline static std::unordered_map<std::string, ScopedTimerAssets> mScopedTimerAssetsMap;

  /// \brief Static vector of active scoped timers.
  inline static std::vector<std::string> mActiveScopedTimers;
};

} // namespace common
} // namespace gls

#endif // GLS_COMMON_SCOPEDTIMER_HPP_
