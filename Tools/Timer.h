/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once

#include <chrono>
#include <cstdint>

// A timer to measure how long some code takes to execute.
class Timer {
 public:
  // Constructs a timer and starts it.
  Timer() : startTime(std::chrono::steady_clock::now()) {}

  // Returns the time elapsed since the timer was started.
  template <typename UnitT = std::chrono::milliseconds>
  int64_t elapsed() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<UnitT>(now - startTime).count();
  }

  // Restarts the timer.
  void restart() {
    startTime = std::chrono::steady_clock::now();
  }

 private:
  std::chrono::steady_clock::time_point startTime; // Time point when the timer was started.
};
