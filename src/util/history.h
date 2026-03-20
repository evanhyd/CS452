#pragma once

#include <array>
#include <cstddef>
#include <iterator>
#include <ranges>

template <typename Entry, size_t N> class History {
public:
  History() : head_{0}, tail_{0} {}

  void push(const Entry& entry) {
    entries_[head_] = entry;
    head_ = (head_ + 1) % N;
    if (head_ == tail_) {
      tail_ = (tail_ + 1) % N;
    }
  }

  bool empty() const { return head_ == tail_; }

  Entry& back() { return entries_[head_ == 0 ? N - 1 : head_ - 1]; }
  const Entry& back() const { return entries_[head_ == 0 ? N - 1 : head_ - 1]; }

  template <typename E>
    requires std::is_same_v<std::remove_const_t<E>, Entry>
  class Iterator {
  public:
    using difference_type = std::ptrdiff_t;
    using value_type = Entry;

    E& operator*() const { return history[idx]; }
    Iterator& operator++() {
      idx = (idx + 1) % N;
      return *this;
    }
    Iterator operator++(int) {
      Iterator temp = *this;
      ++*this;
      return temp;
    }
    Iterator& operator--() {
      idx = (idx + N - 1) % N;
      return *this;
    }
    Iterator operator--(int) {
      Iterator temp = *this;
      --*this;
      return temp;
    }
    bool operator==(const Iterator& other) const { return idx == other.idx; }

    Iterator() : idx{0}, history{nullptr} {}

  private:
    friend History;
    Iterator(size_t i, auto&& h) : idx{i}, history{h.data()} {}
    size_t idx;
    E* history;
  };

  std::bidirectional_iterator auto begin() const { return Iterator<const Entry>{tail_, entries_}; }
  std::bidirectional_iterator auto end() const { return Iterator<const Entry>{head_, entries_}; }
  std::bidirectional_iterator auto cbegin() const { return begin(); }
  std::bidirectional_iterator auto cend() const { return end(); }

  std::bidirectional_iterator auto begin() { return Iterator<Entry>{tail_, entries_}; }
  std::bidirectional_iterator auto end() { return Iterator<Entry>{head_, entries_}; }

  size_t size() const {
    if (head_ >= tail_) {
      return head_ - tail_;
    } else {
      return N - (tail_ - head_);
    }
  }

  static constexpr std::integral_constant<size_t, N> capacity{};

private:
  std::array<Entry, N> entries_;
  size_t head_;
  size_t tail_;
};
static_assert(std::ranges::bidirectional_range<History<int, 2>> &&
              std::ranges::bidirectional_range<const History<int, 2>>);
