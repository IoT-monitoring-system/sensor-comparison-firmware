#pragma once
#ifndef SLIDINGWINDOW_H
#define SLIDINGWINDOW_H

#include <cstddef>
#include <deque>

template <typename T> class SlidingWindow {
private:
  std::deque<T> buffer;
  size_t maxSize;

public:
  SlidingWindow(size_t size) : maxSize(size) {}

  void add(T value) {
    buffer.push_back(value);

    if (buffer.size() > maxSize) {
      buffer.pop_front();
    }
  }

  float getLatest() const { return buffer.empty() ? 0 : buffer.back(); };

  bool isFull() const { return buffer.size() == maxSize; }

  std::deque<float> getQueue() { return buffer; }
};

#endif