#ifndef WAREHOUSE_UTILS_EVENT_BUS_HPP_
#define WAREHOUSE_UTILS_EVENT_BUS_HPP_

#include <functional>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <string>
#include <ros/console.h>

namespace warehouse_utils {

/// Event types for the exploration framework.
/// Modules communicate exclusively through these events (ADR-003).
enum class EventType {
  MAP_READY,             ///< /map has valid data
  SCAN_COMPLETE,         ///< INITIAL_SCAN rotation finished
  EXPLORATION_ENABLED,   ///< MissionManager enables exploration
  EXPLORATION_DONE,      ///< All frontiers exhausted
  NEED_PLAN,             ///< Request A* to plan a path
  PLANNING_SUCCESS,      ///< A* returned a valid path
  PLANNING_FAILED,       ///< A* could not find a path
  GOAL_REACHED,          ///< Robot arrived at goal
  STUCK,                 ///< NavigationMonitor detected stuck
  RECOVERY_REQUESTED,    ///< Request recovery action
  RECOVERY_DONE,         ///< Recovery complete
  ERROR,                 ///< Unrecoverable error
};

/// Convert EventType to string for logging.
inline const char* eventName(EventType e) {
  switch (e) {
    case EventType::MAP_READY:           return "MAP_READY";
    case EventType::SCAN_COMPLETE:       return "SCAN_COMPLETE";
    case EventType::EXPLORATION_ENABLED: return "EXPLORATION_ENABLED";
    case EventType::EXPLORATION_DONE:    return "EXPLORATION_DONE";
    case EventType::NEED_PLAN:           return "NEED_PLAN";
    case EventType::PLANNING_SUCCESS:    return "PLANNING_SUCCESS";
    case EventType::PLANNING_FAILED:     return "PLANNING_FAILED";
    case EventType::GOAL_REACHED:        return "GOAL_REACHED";
    case EventType::STUCK:               return "STUCK";
    case EventType::RECOVERY_REQUESTED:  return "RECOVERY_REQUESTED";
    case EventType::RECOVERY_DONE:       return "RECOVERY_DONE";
    case EventType::ERROR:               return "ERROR";
    default: return "UNKNOWN";
  }
}

/// Simple thread-safe in-process event bus.
///
/// Modules subscribe to event types. When an event is emitted,
/// all subscribers are called in registration order on the emitting thread.
///
/// This is NOT a ROS topic — it's an in-process pub/sub for zero-latency
/// communication between modules in the same process. Use it for FSM
/// transitions, not for inter-node communication.
class EventBus {
public:
  using Callback = std::function<void(EventType)>;

  /// Subscribe to an event type. Returns an opaque subscription ID.
  int subscribe(EventType type, Callback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    int id = next_id_++;
    subscribers_[type].push_back({id, std::move(cb)});
    return id;
  }

  /// Emit an event. All subscribers for this type are called.
  void emit(EventType type) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subscribers_.find(type);
    if (it == subscribers_.end()) return;
    ROS_DEBUG_STREAM("EventBus: emit " << eventName(type)
                     << " → " << it->second.size() << " subscribers");
    for (auto& sub : it->second) {
      sub.cb(type);
    }
  }

  /// Unsubscribe by ID.
  void unsubscribe(int id) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& pair : subscribers_) {
      auto& vec = pair.second;
      vec.erase(std::remove_if(vec.begin(), vec.end(),
        [id](const Sub& s) { return s.id == id; }), vec.end());
    }
  }

  /// Singleton accessor (ADR-006: shared state pattern).
  static EventBus& instance() {
    static EventBus bus;
    return bus;
  }

private:
  struct Sub {
    int id;
    Callback cb;
  };

  std::unordered_map<EventType, std::vector<Sub>> subscribers_;
  std::mutex mutex_;
  int next_id_ = 1;
};

}  // namespace warehouse_utils

#endif  // WAREHOUSE_UTILS_EVENT_BUS_HPP_
