#pragma once

#include <atomic>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace valhalla {
namespace midgard {

struct thread_safe_counter {
  using counter_type = std::atomic<std::size_t>;

  static void increment(counter_type& counter) noexcept {
    counter.fetch_add(1, std::memory_order_relaxed);
  }

  static std::size_t decrement(counter_type& counter) noexcept {
    return counter.fetch_sub(1, std::memory_order_acq_rel) - 1;
  }

  static std::size_t get(const counter_type& counter) noexcept {
    return counter.load(std::memory_order_relaxed);
  }
};

struct thread_unsafe_counter {
  using counter_type = std::size_t;

  static void increment(counter_type& counter) noexcept {
    ++counter;
  }

  static std::size_t decrement(counter_type& counter) noexcept {
    return --counter;
  }

  static std::size_t get(const counter_type& counter) noexcept {
    return counter;
  }
};

template <class T, class CounterPolicy = thread_safe_counter> class intrusive_ref_counter {
protected:
  intrusive_ref_counter() noexcept : ref_count_(0) {
  }

  intrusive_ref_counter(const intrusive_ref_counter&) noexcept : ref_count_(0) {
  }

  intrusive_ref_counter& operator=(const intrusive_ref_counter&) noexcept {
    return *this;
  }

  intrusive_ref_counter(intrusive_ref_counter&&) noexcept : ref_count_(0) {
  }

  intrusive_ref_counter& operator=(intrusive_ref_counter&&) noexcept {
    return *this;
  }

  ~intrusive_ref_counter() = default;

public:
  std::size_t use_count() const noexcept {
    return CounterPolicy::get(ref_count_);
  }

  friend void intrusive_ptr_add_ref(const T* ptr) noexcept {
    auto* self = const_cast<intrusive_ref_counter*>(
        static_cast<const intrusive_ref_counter*>(ptr));
    CounterPolicy::increment(self->ref_count_);
  }

  friend void intrusive_ptr_release(const T* ptr) noexcept {
    auto* self = const_cast<intrusive_ref_counter*>(
        static_cast<const intrusive_ref_counter*>(ptr));
    if (CounterPolicy::decrement(self->ref_count_) == 0) {
      delete const_cast<T*>(static_cast<const T*>(ptr));
    }
  }

private:
  mutable typename CounterPolicy::counter_type ref_count_;
};

template <class T> class intrusive_ptr {
public:
  using element_type = T;

  constexpr intrusive_ptr() noexcept : ptr_(nullptr) {
  }

  intrusive_ptr(T* ptr, bool add_ref = true) : ptr_(ptr) {
    if (ptr_ != nullptr && add_ref) {
      intrusive_ptr_add_ref(ptr_);
    }
  }

  intrusive_ptr(const intrusive_ptr& other) noexcept : ptr_(other.ptr_) {
    if (ptr_ != nullptr) {
      intrusive_ptr_add_ref(ptr_);
    }
  }

  template <class U, class = std::enable_if_t<std::is_convertible<U*, T*>::value>>
  intrusive_ptr(const intrusive_ptr<U>& other) noexcept : ptr_(other.ptr_) {
    if (ptr_ != nullptr) {
      intrusive_ptr_add_ref(ptr_);
    }
  }

  intrusive_ptr(intrusive_ptr&& other) noexcept : ptr_(other.ptr_) {
    other.ptr_ = nullptr;
  }

  template <class U, class = std::enable_if_t<std::is_convertible<U*, T*>::value>>
  intrusive_ptr(intrusive_ptr<U>&& other) noexcept : ptr_(other.ptr_) {
    other.ptr_ = nullptr;
  }

  ~intrusive_ptr() {
    if (ptr_ != nullptr) {
      intrusive_ptr_release(ptr_);
    }
  }

  intrusive_ptr& operator=(intrusive_ptr other) noexcept {
    swap(other);
    return *this;
  }

  template <class U, class = std::enable_if_t<std::is_convertible<U*, T*>::value>>
  intrusive_ptr& operator=(intrusive_ptr<U> other) noexcept {
    swap(other);
    return *this;
  }

  void reset(T* ptr = nullptr, bool add_ref = true) {
    intrusive_ptr(ptr, add_ref).swap(*this);
  }

  T* get() const noexcept {
    return ptr_;
  }

  T& operator*() const {
    return *ptr_;
  }

  T* operator->() const {
    return ptr_;
  }

  explicit operator bool() const noexcept {
    return ptr_ != nullptr;
  }

  void swap(intrusive_ptr& other) noexcept {
    std::swap(ptr_, other.ptr_);
  }

private:
  template <class U> friend class intrusive_ptr;

  T* ptr_;
};

template <class T> inline void swap(intrusive_ptr<T>& lhs, intrusive_ptr<T>& rhs) noexcept {
  lhs.swap(rhs);
}

template <class T> inline T* get_pointer(const intrusive_ptr<T>& p) noexcept {
  return p.get();
}

template <class T, class U>
inline bool operator==(const intrusive_ptr<T>& lhs, const intrusive_ptr<U>& rhs) noexcept {
  return lhs.get() == rhs.get();
}

template <class T, class U>
inline bool operator!=(const intrusive_ptr<T>& lhs, const intrusive_ptr<U>& rhs) noexcept {
  return !(lhs == rhs);
}

template <class T> inline bool operator==(const intrusive_ptr<T>& lhs, std::nullptr_t) noexcept {
  return !lhs;
}

template <class T> inline bool operator==(std::nullptr_t, const intrusive_ptr<T>& rhs) noexcept {
  return !rhs;
}

template <class T> inline bool operator!=(const intrusive_ptr<T>& lhs, std::nullptr_t) noexcept {
  return static_cast<bool>(lhs);
}

template <class T> inline bool operator!=(std::nullptr_t, const intrusive_ptr<T>& rhs) noexcept {
  return static_cast<bool>(rhs);
}

template <class T, class U>
intrusive_ptr<T> static_pointer_cast(const intrusive_ptr<U>& p) {
  return intrusive_ptr<T>(static_cast<T*>(p.get()));
}

template <class T, class U>
intrusive_ptr<T> const_pointer_cast(const intrusive_ptr<U>& p) {
  return intrusive_ptr<T>(const_cast<T*>(p.get()));
}

template <class T, class U>
intrusive_ptr<T> dynamic_pointer_cast(const intrusive_ptr<U>& p) {
  if (auto* casted = dynamic_cast<T*>(p.get())) {
    return intrusive_ptr<T>(casted);
  }
  return intrusive_ptr<T>();
}

} // namespace midgard
} // namespace valhalla

namespace boost {

using valhalla::midgard::intrusive_ptr;

template <class T, class CounterPolicy = valhalla::midgard::thread_safe_counter>
using intrusive_ref_counter = valhalla::midgard::intrusive_ref_counter<T, CounterPolicy>;

using valhalla::midgard::thread_safe_counter;
using valhalla::midgard::thread_unsafe_counter;

using valhalla::midgard::const_pointer_cast;
using valhalla::midgard::dynamic_pointer_cast;
using valhalla::midgard::static_pointer_cast;

} // namespace boost
