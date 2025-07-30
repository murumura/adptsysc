#pragma once
#include <adptsysc/common.hh>

namespace adptsysc {

template <typename T>
class SharedQueue {
 public:
  bool empty() const {
    std::lock_guard lock{mutex};
    return queue.empty();
  }

  std::size_t size() const {
    std::lock_guard lock{mutex};
    return queue.size();
  }

  void push(T new_elem) {
    std::lock_guard lock{mutex};
    queue.push_back(new_elem);
    condvar.notify_one();
  }

  T wait_and_pop() {
    std::unique_lock lock{mutex};

    while (queue.empty()) {
      condvar.wait(lock);
    }

    T result = std::move(queue.front());
    queue.pop_front();

    return result;
  }

  std::optional<T> try_pop() {
    std::unique_lock lock{mutex};

    if (queue.empty()) {
      return {};
    }

    T result = std::move(queue.front());
    queue.pop_front();

    return result;
  }

 private:
  std::deque<T> queue;
  mutable std::mutex mutex;
  std::condition_variable condvar;
};

}  // namespace adptsysc