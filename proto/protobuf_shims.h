// Helper shims that emulate a subset of the generated protobuf container API.
#ifndef VALHALLA_CUSTOM_WASM_SHIMS_H_
#define VALHALLA_CUSTOM_WASM_SHIMS_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <type_traits>
#include <utility>
#include <vector>

namespace valhalla {
namespace proto {

template <typename T> class RepeatedFieldShim {
public:
  using Storage = std::vector<T>;
  using value_type = T;
  using reference = typename Storage::reference;
  using const_reference = typename Storage::const_reference;
  using iterator = typename Storage::iterator;
  using const_iterator = typename Storage::const_iterator;
  using reverse_iterator = typename Storage::reverse_iterator;
  using const_reverse_iterator = typename Storage::const_reverse_iterator;

  RepeatedFieldShim(Storage &storage) : storage_(&storage) {}

  void Reset(Storage *storage) { storage_ = storage; }

  RepeatedFieldShim *operator->() { return this; }
  const RepeatedFieldShim *operator->() const { return this; }

  iterator begin() { return storage_->begin(); }
  const_iterator begin() const { return storage_->begin(); }
  const_iterator cbegin() const { return storage_->cbegin(); }

  iterator end() { return storage_->end(); }
  const_iterator end() const { return storage_->end(); }
  const_iterator cend() const { return storage_->cend(); }

  reverse_iterator rbegin() { return storage_->rbegin(); }
  const_reverse_iterator rbegin() const { return storage_->rbegin(); }
  const_reverse_iterator crbegin() const { return storage_->crbegin(); }

  reverse_iterator rend() { return storage_->rend(); }
  const_reverse_iterator rend() const { return storage_->rend(); }
  const_reverse_iterator crend() const { return storage_->crend(); }

  std::size_t size() const { return storage_->size(); }
  bool empty() const { return storage_->empty(); }

  void Clear() { storage_->clear(); }
  void clear() { Clear(); }

  void Reserve(std::size_t count) { storage_->reserve(count); }

  void DeleteSubrange(int start, int count) {
    if (count <= 0) {
      return;
    }
    auto first = storage_->begin() + start;
    storage_->erase(first, first + count);
  }

  void RemoveLast() { storage_->pop_back(); }

  void SwapElements(int index1, int index2) {
    using std::swap;
    swap((*storage_)[index1], (*storage_)[index2]);
  }

  T *Add() {
    storage_->emplace_back();
    return &storage_->back();
  }

  void Add(const T &value) { storage_->push_back(value); }

  void Add(T &&value) { storage_->push_back(std::move(value)); }

  template <class... Args> T *EmplaceBack(Args &&...args) {
    storage_->emplace_back(std::forward<Args>(args)...);
    return &storage_->back();
  }

  void MergeFrom(const Storage &other) {
    storage_->insert(storage_->end(), other.begin(), other.end());
  }

  void CopyFrom(const Storage &other) { *storage_ = other; }

  T *Mutable(int index) { return &(*storage_)[index]; }

  const T &Get(int index) const { return (*storage_)[index]; }

  void Set(int index, const T &value) { (*storage_)[index] = value; }

  void Resize(std::size_t count) { storage_->resize(count); }

  void Resize(std::size_t count, const T &value) {
    storage_->resize(count, value);
  }

  T &operator[](std::size_t index) { return (*storage_)[index]; }
  const T &operator[](std::size_t index) const { return (*storage_)[index]; }

  Storage *storage() { return storage_; }
  const Storage &storage() const { return *storage_; }

  RepeatedFieldShim &operator=(const RepeatedFieldShim &other) {
    if (this != &other) {
      *storage_ = *(other.storage_);
    }
    return *this;
  }

  RepeatedFieldShim &operator=(RepeatedFieldShim &&other) {
    if (this != &other) {
      *storage_ = std::move(*(other.storage_));
    }
    return *this;
  }

  RepeatedFieldShim &operator=(const Storage &other) {
    *storage_ = other;
    return *this;
  }

  RepeatedFieldShim &operator=(Storage &&other) {
    *storage_ = std::move(other);
    return *this;
  }

private:
  Storage *storage_;
};

template <> class RepeatedFieldShim<bool> {
public:
  using Storage = std::vector<bool>;
  using value_type = bool;
  using reference = Storage::reference;
  using const_reference = bool;
  using iterator = Storage::iterator;
  using const_iterator = Storage::const_iterator;
  using reverse_iterator = Storage::reverse_iterator;
  using const_reverse_iterator = Storage::const_reverse_iterator;

  RepeatedFieldShim(Storage &storage) : storage_(&storage) {}

  void Reset(Storage *storage) { storage_ = storage; }

  RepeatedFieldShim *operator->() { return this; }
  const RepeatedFieldShim *operator->() const { return this; }

  iterator begin() { return storage_->begin(); }
  const_iterator begin() const { return storage_->begin(); }
  const_iterator cbegin() const { return storage_->cbegin(); }

  iterator end() { return storage_->end(); }
  const_iterator end() const { return storage_->end(); }
  const_iterator cend() const { return storage_->cend(); }

  reverse_iterator rbegin() { return storage_->rbegin(); }
  const_reverse_iterator rbegin() const { return storage_->rbegin(); }
  const_reverse_iterator crbegin() const { return storage_->crbegin(); }

  reverse_iterator rend() { return storage_->rend(); }
  const_reverse_iterator rend() const { return storage_->rend(); }
  const_reverse_iterator crend() const { return storage_->crend(); }

  std::size_t size() const { return storage_->size(); }
  bool empty() const { return storage_->empty(); }

  void Clear() { storage_->clear(); }
  void clear() { Clear(); }

  void Reserve(std::size_t count) { storage_->reserve(count); }

  void DeleteSubrange(int start, int count) {
    if (count <= 0) {
      return;
    }
    auto first = storage_->begin() + start;
    storage_->erase(first, first + count);
  }

  void RemoveLast() { storage_->pop_back(); }

  void SwapElements(int index1, int index2) {
    bool value1 = (*storage_)[index1];
    bool value2 = (*storage_)[index2];
    (*storage_)[index1] = value2;
    (*storage_)[index2] = value1;
  }

  void Add(bool value) { storage_->push_back(value); }

  void MergeFrom(const Storage &other) {
    storage_->insert(storage_->end(), other.begin(), other.end());
  }

  void CopyFrom(const Storage &other) { *storage_ = other; }

  reference Mutable(int index) { return (*storage_)[index]; }

  bool Get(int index) const { return (*storage_)[index]; }

  void Set(int index, bool value) { (*storage_)[index] = value; }

  void Resize(std::size_t count) { storage_->resize(count); }

  void Resize(std::size_t count, bool value) { storage_->resize(count, value); }

  reference operator[](std::size_t index) { return (*storage_)[index]; }
  bool operator[](std::size_t index) const { return (*storage_)[index]; }

  Storage *storage() { return storage_; }
  const Storage &storage() const { return *storage_; }

private:
  Storage *storage_;
};

template <typename K, typename V> class MapFieldShim {
public:
  using Storage = std::map<K, V>;
  using iterator = typename Storage::iterator;
  using const_iterator = typename Storage::const_iterator;

  MapFieldShim(Storage &storage) : storage_(&storage) {}

  void Reset(Storage *storage) { storage_ = storage; }

  MapFieldShim *operator->() { return this; }
  const MapFieldShim *operator->() const { return this; }

  iterator begin() { return storage_->begin(); }
  const_iterator begin() const { return storage_->begin(); }
  const_iterator cbegin() const { return storage_->cbegin(); }

  iterator end() { return storage_->end(); }
  const_iterator end() const { return storage_->end(); }
  const_iterator cend() const { return storage_->cend(); }

  bool empty() const { return storage_->empty(); }
  std::size_t size() const { return storage_->size(); }

  void Clear() { storage_->clear(); }
  void clear() { Clear(); }

  std::pair<iterator, bool> insert(std::pair<K, V> value) {
    return storage_->insert(std::move(value));
  }

  template <class... Args> std::pair<iterator, bool> emplace(Args &&...args) {
    return storage_->emplace(std::forward<Args>(args)...);
  }

  V &operator[](const K &key) { return (*storage_)[key]; }

  V &operator[](K &&key) { return (*storage_)[std::move(key)]; }

  template <
      typename Key, typename Decayed = typename std::decay<Key>::type,
      typename = typename std::enable_if<
          !std::is_same<Decayed, K>::value &&
          (std::is_convertible<Key, K>::value ||
           (std::is_enum<Decayed>::value &&
            std::is_convertible<typename std::underlying_type<Decayed>::type,
                                K>::value))>::type>
  V &operator[](Key &&key) {
    return (*storage_)[ConvertKey(std::forward<Key>(key))];
  }

  iterator find(const K &key) { return storage_->find(key); }
  const_iterator find(const K &key) const { return storage_->find(key); }

  template <
      typename Key, typename Decayed = typename std::decay<Key>::type,
      typename = typename std::enable_if<
          !std::is_same<Decayed, K>::value &&
          (std::is_convertible<Key, K>::value ||
           (std::is_enum<Decayed>::value &&
            std::is_convertible<typename std::underlying_type<Decayed>::type,
                                K>::value))>::type>
  iterator find(Key &&key) {
    return storage_->find(ConvertKey(std::forward<Key>(key)));
  }

  template <
      typename Key, typename Decayed = typename std::decay<Key>::type,
      typename = typename std::enable_if<
          !std::is_same<Decayed, K>::value &&
          (std::is_convertible<Key, K>::value ||
           (std::is_enum<Decayed>::value &&
            std::is_convertible<typename std::underlying_type<Decayed>::type,
                                K>::value))>::type>
  const_iterator find(Key &&key) const {
    return storage_->find(ConvertKey(std::forward<Key>(key)));
  }

  std::size_t erase(const K &key) { return storage_->erase(key); }

  void MergeFrom(const Storage &other) {
    storage_->insert(other.begin(), other.end());
  }

  void CopyFrom(const Storage &other) { *storage_ = other; }

  Storage *storage() { return storage_; }
  const Storage &storage() const { return *storage_; }

private:
  Storage *storage_;
  template <typename Key> static K ConvertKey(Key &&key) {
    using Decayed = typename std::decay<Key>::type;
    if constexpr (std::is_enum<Decayed>::value) {
      using Underlying = typename std::underlying_type<Decayed>::type;
      return static_cast<K>(static_cast<Underlying>(key));
    } else {
      return static_cast<K>(std::forward<Key>(key));
    }
  }
};

} // namespace proto

template <typename Enum, typename Integral,
          typename std::enable_if<std::is_enum<Enum>::value &&
                                      std::is_integral<Integral>::value,
                                  int>::type = 0>
constexpr bool operator==(Enum lhs, Integral rhs) noexcept {
  using Underlying = typename std::underlying_type<Enum>::type;
  return static_cast<Underlying>(lhs) == static_cast<Underlying>(rhs);
}

template <typename Integral, typename Enum,
          typename std::enable_if<std::is_integral<Integral>::value &&
                                      std::is_enum<Enum>::value,
                                  int>::type = 0>
constexpr bool operator==(Integral lhs, Enum rhs) noexcept {
  return rhs == lhs;
}

template <typename Enum, typename Integral,
          typename std::enable_if<std::is_enum<Enum>::value &&
                                      std::is_integral<Integral>::value,
                                  int>::type = 0>
constexpr bool operator!=(Enum lhs, Integral rhs) noexcept {
  return !(lhs == rhs);
}

template <typename Integral, typename Enum,
          typename std::enable_if<std::is_integral<Integral>::value &&
                                      std::is_enum<Enum>::value,
                                  int>::type = 0>
constexpr bool operator!=(Integral lhs, Enum rhs) noexcept {
  return !(lhs == rhs);
}
} // namespace valhalla

#endif // VALHALLA_CUSTOM_WASM_SHIMS_H_
