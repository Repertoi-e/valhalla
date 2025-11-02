#pragma once

#include "ptree_fwd.hpp"

#include <algorithm>
#include <cctype>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <midgard/string_utils.h>

namespace valhalla {

/**
 * A lightweight drop-in replacement for Boost.PropertyTree's property_tree that stores
 * configuration data in a simple ordered tree backed by standard containers.
 *
 * The implementation focuses on the subset of the original interface that
 * Valhalla relies on and intentionally keeps the surface area minimal. It keeps
 * the namespace and class name identical so existing code continues to compile
 * without modification, while internally reusing RapidJSON for parsing.
 */
class property_tree {
public:
  using key_type = std::string;
  using data_type = std::string;
  using mapped_type = property_tree;
  using sequence_type = std::vector<std::pair<key_type, mapped_type>>;
  using value_type = sequence_type::value_type;
  using iterator = sequence_type::iterator;
  using const_iterator = sequence_type::const_iterator;
  using size_type = sequence_type::size_type;
  using assoc_iterator = iterator;
  using const_assoc_iterator = const_iterator;

  template <typename T> class optional_ref {
  public:
    optional_ref() noexcept = default;
    optional_ref(T& ref) noexcept : ptr_(std::addressof(ref)) {}
    optional_ref(T* ptr) noexcept : ptr_(ptr) {}

    bool has_value() const noexcept {
      return ptr_ != nullptr;
    }

    explicit operator bool() const noexcept {
      return has_value();
    }

    T& get() const {
      if (!ptr_) {
        throw std::runtime_error("Accessing empty optional_ref");
      }
      return *ptr_;
    }

    T& value() const {
      return get();
    }

    T* operator->() const noexcept {
      return ptr_;
    }

    T& operator*() const {
      return get();
    }

    void reset() noexcept {
      ptr_ = nullptr;
    }

  private:
    T* ptr_ = nullptr;
  };

  using optional_child = optional_ref<property_tree>;
  using optional_const_child = optional_ref<const property_tree>;

  property_tree() = default;
  ~property_tree() = default;
  property_tree(const property_tree&) = default;
  property_tree(property_tree&&) noexcept = default;
  property_tree& operator=(const property_tree&) = default;
  property_tree& operator=(property_tree&&) noexcept = default;

  bool empty() const noexcept {
    return children_.empty();
  }

  size_type size() const noexcept {
    return children_.size();
  }

  void clear() noexcept {
    data_.clear();
    children_.clear();
  }

  data_type& data() noexcept {
    return data_;
  }

  const data_type& data() const noexcept {
    return data_;
  }

  iterator begin() noexcept {
    return children_.begin();
  }

  iterator end() noexcept {
    return children_.end();
  }

  const_iterator begin() const noexcept {
    return children_.begin();
  }

  const_iterator end() const noexcept {
    return children_.end();
  }

  const_iterator cbegin() const noexcept {
    return children_.cbegin();
  }

  const_iterator cend() const noexcept {
    return children_.cend();
  }

  assoc_iterator find(const key_type& key) {
    return find_child_iter(key);
  }

  const_assoc_iterator find(const key_type& key) const {
    return find_child_iter(key);
  }

  assoc_iterator not_found() noexcept {
    return children_.end();
  }

  const_assoc_iterator not_found() const noexcept {
    return children_.cend();
  }

  iterator to_iterator(assoc_iterator it) noexcept {
    return it;
  }

  const_iterator to_iterator(const_assoc_iterator it) const noexcept {
    return it;
  }

  template <typename T> void put(const key_type& path, const T& value) {
    auto& node = ensure_node(path);
    node.put_value(value);
  }

  template <typename T>
  T get(const key_type& path, const T& default_value) const {
    const property_tree* node = find_node(path);
    if (!node) {
      return default_value;
    }
    return node->template get_value<T>(default_value);
  }

  template <typename T> T get(const key_type& path) const {
    const property_tree* node = find_node(path);
    if (!node) {
      throw std::runtime_error("Path not found: " + path);
    }
    return node->template get_value<T>();
  }

  template <typename T>
  std::optional<T> get_optional(const key_type& path) const {
    const property_tree* node = find_node(path);
    if (!node) {
      return std::nullopt;
    }
    try {
      return node->template get_value<T>();
    } catch (...) {
      return std::nullopt;
    }
  }

  size_t count(const key_type& key) const {
    size_t cnt = 0;
    for (const auto& child : children_) {
      if (child.first == key) {
        ++cnt;
      }
    }
    return cnt;
  }

  property_tree& get_child(const key_type& path) {
    property_tree* node = find_node(path);
    if (!node) {
      throw std::runtime_error("Path not found: " + path);
    }
    return *node;
  }

  const property_tree& get_child(const key_type& path) const {
    const property_tree* node = find_node(path);
    if (!node) {
      throw std::runtime_error("Path not found: " + path);
    }
    return *node;
  }

  optional_child get_child_optional(const key_type& path) {
    return optional_child(find_node(path));
  }

  optional_const_child get_child_optional(const key_type& path) const {
    return optional_const_child(find_node(path));
  }

  template <typename T> property_tree& add(const key_type& path, T&& value) {
    property_tree& child = add_child(path, property_tree{});
    child.put_value(std::forward<T>(value));
    return child;
  }

  property_tree& add_child(const key_type& path, const property_tree& child) {
    return add_child_impl(path, child);
  }

  property_tree& add_child(const key_type& path, property_tree&& child) {
    return add_child_impl(path, std::move(child));
  }

  iterator push_back(const value_type& value) {
    children_.push_back(value);
    return std::prev(children_.end());
  }

  iterator push_back(value_type&& value) {
    children_.push_back(std::move(value));
    return std::prev(children_.end());
  }

  value_type& front() {
    if (children_.empty()) {
      throw std::out_of_range("property_tree::front on empty tree");
    }
    return children_.front();
  }

  const value_type& front() const {
    if (children_.empty()) {
      throw std::out_of_range("property_tree::front on empty tree");
    }
    return children_.front();
  }

  size_type erase(const key_type& key) {
    size_type removed = 0;
    auto it = children_.begin();
    while (it != children_.end()) {
      if (it->first == key) {
        it = children_.erase(it);
        ++removed;
      } else {
        ++it;
      }
    }
    return removed;
  }

  iterator erase(const_iterator pos) {
    return children_.erase(pos);
  }

  template <typename T> T get_value() const {
    if constexpr (std::is_same_v<T, std::string>) {
      return data_;
    } else if constexpr (std::is_same_v<T, bool>) {
      auto lower = midgard::string_utils::to_lower_copy(data_);
      if (lower == "true" || lower == "1") {
        return true;
      }
      if (lower == "false" || lower == "0") {
        return false;
      }
      throw std::runtime_error("Failed to convert value to bool: " + data_);
    } else if constexpr (std::is_arithmetic_v<T>) {
      if (data_.empty()) {
        throw std::runtime_error("No data stored in node");
      }
      std::istringstream stream(data_);
      stream.unsetf(std::ios::skipws);
      T result{};
      if (!(stream >> result) || !stream.eof()) {
        throw std::runtime_error("Failed to convert value: " + data_);
      }
      return result;
    } else {
      static_assert(!sizeof(T), "Unsupported type for property_tree::get_value");
    }
  }

  template <typename T> T get_value(const T& default_value) const {
    if (data_.empty()) {
      return default_value;
    }
    try {
      return get_value<T>();
    } catch (...) {
      return default_value;
    }
  }

  template <typename T> void put_value(T&& value) {
    data_ = to_string(std::forward<T>(value));
  }

private:
  static std::vector<key_type> split(const key_type& path) {
    std::vector<key_type> segments;
    if (path.empty()) {
      return segments;
    }
    key_type current;
    for (char c : path) {
      if (c == '.') {
        segments.push_back(current);
        current.clear();
      } else {
        current.push_back(c);
      }
    }
    segments.push_back(current);
    return segments;
  }

  template <typename T>
  static std::string to_string(T&& value) {
    using Decayed = std::decay_t<T>;
    if constexpr (std::is_same_v<Decayed, std::string>) {
      return std::forward<T>(value);
    } else if constexpr (std::is_same_v<Decayed, const char*> || std::is_same_v<Decayed, char*>) {
      return std::string(value);
    } else if constexpr (std::is_same_v<Decayed, bool>) {
      return value ? "true" : "false";
    } else if constexpr (std::is_arithmetic_v<Decayed>) {
      if constexpr (std::is_floating_point_v<Decayed>) {
        std::ostringstream stream;
        stream << value;
        return stream.str();
      } else {
        return std::to_string(value);
      }
    } else {
      std::ostringstream stream;
      stream << value;
      return stream.str();
    }
  }

  property_tree& ensure_node(const key_type& path) {
    if (path.empty()) {
      return *this;
    }
    auto segments = split(path);
    property_tree* current = this;
    for (const auto& segment : segments) {
      auto it = current->find_child_iter(segment);
      if (it == current->children_.end()) {
        current->children_.emplace_back(segment, property_tree{});
        it = std::prev(current->children_.end());
      }
      current = &it->second;
    }
    return *current;
  }

  const property_tree* find_node(const key_type& path) const {
    if (path.empty()) {
      return this;
    }
    auto segments = split(path);
    const property_tree* current = this;
    for (const auto& segment : segments) {
      auto it = current->find_child_iter(segment);
      if (it == current->children_.cend()) {
        return nullptr;
      }
      current = &it->second;
    }
    return current;
  }

  property_tree* find_node(const key_type& path) {
    return const_cast<property_tree*>(static_cast<const property_tree*>(this)->find_node(path));
  }

  const_iterator find_child_iter(const key_type& key) const {
    return std::find_if(children_.cbegin(), children_.cend(), [&key](const value_type& child) {
      return child.first == key;
    });
  }

  iterator find_child_iter(const key_type& key) {
    return std::find_if(children_.begin(), children_.end(), [&key](const value_type& child) {
      return child.first == key;
    });
  }

  template <typename Child>
  property_tree& add_child_impl(const key_type& path, Child&& child) {
    if (path.empty()) {
      children_.emplace_back("", std::forward<Child>(child));
      return children_.back().second;
    }
    auto segments = split(path);
    if (segments.empty()) {
      children_.emplace_back("", std::forward<Child>(child));
      return children_.back().second;
    }
    property_tree* parent = this;
    if (segments.size() > 1) {
      for (size_t i = 0; i < segments.size() - 1; ++i) {
        auto it = parent->find_child_iter(segments[i]);
        if (it == parent->children_.end()) {
          parent->children_.emplace_back(segments[i], property_tree{});
          it = std::prev(parent->children_.end());
        }
        parent = &it->second;
      }
    }
    const auto& final_key = segments.back();
    parent->children_.emplace_back(final_key, std::forward<Child>(child));
    return parent->children_.back().second;
  }

  data_type data_;
  sequence_type children_;
};

} // namespace valhalla
