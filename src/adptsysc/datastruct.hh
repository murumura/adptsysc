#include <adptsysc/common.hh>

namespace adptsysc {
template <typename T>
class SharedQueue {
 public:
  bool empty() const {
    std::lock_guard lock{mutex};
    return raw_queue.empty();
  }

  std::size_t size() const {
    std::lock_guard lock{mutex};
    return raw_queue.size();
  }

  void push(T newElem) {
    std::lock_guard lock{mutex};
    raw_queue.push_back(newElem);
    cond_var.notify_one();
  }

  T wait_and_pop() {
    std::unique_lock lock{mutex};

    while (raw_queue.empty()) {
      cond_var.wait(lock);
    }

    T result = std::move(raw_queue.front());
    raw_queue.pop_front();

    return result;
  }

  std::optional<T> try_pop() {
    std::unique_lock lock{mutex};

    if (raw_queue.empty()) {
      return {};
    }

    T result = std::move(raw_queue.front());
    raw_queue.pop_front();

    return result;
  }

 private:
  std::deque<T> raw_queue;
  mutable std::mutex mutex;
  std::condition_variable cond_var;
};
}  // namespace adptsysc