#include "hc_tj_marvin_bridge_cpp/marvin_sdk_lock.hpp"

namespace hc_tj_marvin_bridge {

std::recursive_mutex &MarvinSdkLock::mutex() {
  static std::recursive_mutex instance;
  return instance;
}

MarvinSdkLock::MarvinSdkLock(bool try_only)
    : lock_(mutex(), std::defer_lock) {
  if (try_only) {
    lock_.try_lock();
  } else {
    lock_.lock();
  }
}

bool MarvinSdkLock::acquired() const { return lock_.owns_lock(); }

}  // namespace hc_tj_marvin_bridge
