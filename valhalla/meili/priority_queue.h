// -*- mode: c++ -*-
#ifndef MMP_PRIORITY_QUEUE_H_
#define MMP_PRIORITY_QUEUE_H_

#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

// Shortest-path-specific priority queue using a binary heap with lazy duplicate pruning.
template <typename T> class SPQueue {
public:
  // Min heap
  using Heap = std::priority_queue<T, std::vector<T>, std::greater<T>>;

  ~SPQueue() {
    clear();
  }

  void push(const T& label) {
    const auto& id = label.id();
    auto it = best_.find(id);
    if (it == best_.end()) {
      best_.emplace(id, label);
      heap_.push(label);
    } else if (label < it->second) {
      it->second = label;
      heap_.push(label);
    }
  }

  void pop() {
    prune();
    if (heap_.empty()) {
      return;
    }
    best_.erase(heap_.top().id());
    heap_.pop();
  }

  const T& top() const {
    const_cast<SPQueue*>(this)->prune();
    return heap_.top();
  }

  bool empty() const {
    const_cast<SPQueue*>(this)->prune();
    return heap_.empty();
  }

  void clear() {
    Heap empty_heap;
    heap_.swap(empty_heap);
    best_.clear();
  }

  typename Heap::size_type size() const {
    return static_cast<typename Heap::size_type>(best_.size());
  }

protected:
  void prune() {
    while (!heap_.empty()) {
      const auto& candidate = heap_.top();
      auto it = best_.find(candidate.id());
      if (it == best_.end() || !equivalent(candidate, it->second)) {
        heap_.pop();
        continue;
      }
      break;
    }
  }

  static bool equivalent(const T& lhs, const T& rhs) {
    return !(lhs < rhs) && !(rhs < lhs);
  }

  Heap heap_;
  std::unordered_map<typename T::id_type, T> best_;
};

#endif // MMP_PRIORITY_QUEUE_H_
