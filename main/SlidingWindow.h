#pragma once
#ifndef SLIDINGWINDOW_H
#define SLIDINGWINDOW_H

#include <cstddef>
#include <deque>

class SlidingWindow {
private:
  std::deque<float> buffer;
  size_t maxSize;
  float sum = 0;

public:
  SlidingWindow(size_t size);

  void add(float value);

  float mean() const;

  float getLatest() const;

  bool isFull() const;

  std::deque<float> getQueue();
};

#endif