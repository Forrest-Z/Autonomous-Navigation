
#include <algorithm>
#include <deque>
#include <utility>

#ifndef MUTABLE_QUEUE
#define MUTABLE_QUEUE

using std::deque;
using std::pair;
using std::make_pair;

template<class Value, class Priority>
class SimpleQueue {
 private:
  public:
  // Insert a new value, with the specified priority. If the value
  // already exists, its priority is updated.
  void Push(const Value& v, const Priority& p) {
    for (auto& x : values_) {
      // If the value already exists, update its priority, re-sort the priority
      // queue, and return.
      if (x.first == v) {
        x.second = p;
        Sort();
        return;
      }
    }
    // Find where this value should go, and insert it there.
    for (size_t i = 0; i < values_.size(); ++i) {
      if (values_[i].second > p) {
        values_.insert(values_.begin() + i, make_pair(v, p));
        return;
      }
    }
    values_.insert(values_.end(), make_pair(v, p));
  }

  // Sorts the priorities.
  void Sort() {
    static const auto comparator = 
        [](const pair<Value, Priority>& v1, const pair<Value, Priority>& v2) {
      return (v1.second > v2.second);
    };
    sort(values_.begin(), values_.end(), comparator);
  }

  // Retreive the value with the highest priority.
  Value Pop() {
    if (values_.empty()) {
      fprintf(stderr, "ERROR: Pop() called on an empty queue!\n");
      exit(1);
    }
    Sort();
    const Value v = values_.back().first;
    values_.resize(values_.size() - 1);
    return v;
  }

  // Returns true iff the priority queue is empty.
  bool Empty() {
    return values_.empty();
  }

  // Returns true iff the provided value is already on the queue.
  bool Exists(const Value& v) {
    for (const auto& x : values_) {
      if (x.first == v) return true;
    }
    return false;
  }

  private:
  deque<pair<Value, Priority> > values_;
};

#endif  // MUTABLE_QUEUE
