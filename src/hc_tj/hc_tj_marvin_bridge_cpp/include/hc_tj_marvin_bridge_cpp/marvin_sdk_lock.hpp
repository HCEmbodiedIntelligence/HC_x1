#pragma once

#include <mutex>

namespace hc_tj_marvin_bridge {

// Marvin SDK 非线程安全；控制线程与回零线程须串行化 SDK 调用
class MarvinSdkLock {
 public:
  explicit MarvinSdkLock(bool try_only = false);
  MarvinSdkLock(const MarvinSdkLock &) = delete;
  MarvinSdkLock &operator=(const MarvinSdkLock &) = delete;

  bool acquired() const;

 private:
  static std::recursive_mutex &mutex();

  std::unique_lock<std::recursive_mutex> lock_;
};

}  // namespace hc_tj_marvin_bridge
