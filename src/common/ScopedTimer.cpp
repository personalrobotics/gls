/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/common/ScopedTimer.hpp"

#include <utility>

namespace gls {

// Make available the static variables in the source file.
std::unordered_map<std::string, ScopedTimer> ScopedTimer::mScopedTimersMap;
std::vector<ScopedTimer> ScopedTimer::mActiveScopedTimers;

// ================================================================================================
ScopedTimer::ScopedTimer(const std::string name) {
  mName = name;
  if (mActiveScopedTimers.empty()) {
    mParent = "None";
  } else {
    mParent = mActiveScopedTimers.back().getName();
    mNestedLevel = mActiveScopedTimers.back().getNestedLevel() + 1;
  }
  mStartTime = std::chrono::system_clock::now();
}

// ================================================================================================
ScopedTimer::~ScopedTimer() {
  std::chrono::time_point<std::chrono::system_clock> endTime(
      std::chrono::system_clock::now());
  std::chrono::duration<double> totalTime{endTime - mStartTime};

  auto iter = mScopedTimersMap.find(mName);
  if (iter == mScopedTimersMap.end()) {
    mScopedTimersMap.insert(std::pair<std::string, ScopedTimer>(mName, *this));
  } else {
    (*iter).second.mTotalTime += totalTime.count();
    (*iter).second.mCount += 1;
  }
}

// ================================================================================================
const std::unordered_map<std::string, ScopedTimer>
ScopedTimer::getScopedTimersMap() {
  return mScopedTimersMap;
}

// ================================================================================================
std::string ScopedTimer::getName() const { return mName; }

// ================================================================================================
std::size_t ScopedTimer::getCount() const { return mCount; }

// ================================================================================================
double ScopedTimer::getCurrentTime() const {
  std::chrono::time_point<std::chrono::system_clock> currentTime(
      std::chrono::system_clock::now());
  std::chrono::duration<double> totalTime{currentTime - mStartTime};
  return totalTime.count();
}

// ================================================================================================
double ScopedTimer::getTotalTime() const { return mTotalTime; }

// ================================================================================================
std::string ScopedTimer::getParentName() const { return mParent; }

// ================================================================================================
std::size_t ScopedTimer::getNestedLevel() const { return mNestedLevel; }

}  // namespace gls
