#include "SlidingWindow.h"

SlidingWindow::SlidingWindow(size_t size) : maxSize(size) {}

void SlidingWindow::add(float value) {
  buffer.push_back(value);
  sum += value;

  if (buffer.size() > maxSize) {
    sum -= buffer.front();
    buffer.pop_front();
  }
}

float SlidingWindow::mean() const {
  return buffer.empty() ? 0 : sum / buffer.size();
}

float SlidingWindow::getLatest() const {
  return buffer.empty() ? 0 : buffer.back();
}

bool SlidingWindow::isFull() const { return buffer.size() == maxSize; }

std::deque<float> SlidingWindow::getQueue() {
  return buffer;
}
