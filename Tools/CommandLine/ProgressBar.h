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

#include <cassert>
#include <cstdint>
#include <iostream>
#include <ostream>

#include <omp.h>

// A textual indicator of progress towards some goal.
class ProgressBar {
 public:
  // Constructs an uninitialized progress bar.
  explicit ProgressBar(bool verbose = true, std::ostream& os = std::cout)
      : os(os), numSteps(0), stepsDone(0), percentageDone(0),
        percentageOutputInterval(20), dotOutputInterval(5), verbose(verbose) {}

  // Constructs a progress bar with the specified number of steps.
  template <typename ArithmeticT>
  explicit ProgressBar(ArithmeticT numSteps, bool verbose = true, std::ostream& os = std::cout)
      : ProgressBar(verbose, os) {
    init(numSteps);
  }

  // Initialize the progress bar with the specified number of steps.
  void init(const int64_t steps) {
    assert(steps >= 0);
    numSteps = steps;
    stepsDone = 0;
    percentageDone = 0;
    if (verbose)
      os << "0% " << std::flush;
  }

  // Set the percentage points between two printed percentages.
  void setPercentageOutputInterval(const int points) {
    percentageOutputInterval = points;
  }

  // Set the percentage points between two printed dots.
  void setDotOutputInterval(const int points) {
    dotOutputInterval = points;
  }

  // Advances the progress bar to the specified step.
  void advanceTo(const int64_t step) {
    if (!verbose)
      return;
    assert(step >= stepsDone); assert(step <= numSteps);
    stepsDone = step;
    print(stepsDone * 100 / numSteps);
  }

  // Advances the progress bar to 100 %.
  void finish() {
    advanceTo(numSteps);
  }

  // Advances the progress bar by one step.
  void operator++() {
    if (!verbose)
      return;
    int64_t done;
    #pragma omp atomic capture
    done = ++stepsDone;
#ifdef _OPENMP
    if (omp_get_thread_num() == 0)
#endif
      print(done * 100 / numSteps);
  }

  // Advances the progress bar by the specified number of steps.
  void operator+=(const int64_t steps) {
    assert(steps >= 0);
    if (!verbose)
      return;
    int64_t done;
    #pragma omp atomic capture
    done = stepsDone += steps;
#ifdef _OPENMP
    if (omp_get_thread_num() == 0)
#endif
      print(done * 100 / numSteps);
  }

 private:
  // Prints the progress bar until the specified percentage.
  void print(const int until) {
    assert(until <= 100);
    for (int i = percentageDone + 1; i <= until; ++i)
      if (i % percentageOutputInterval == 0)
        os << " " << i << "% " << std::flush;
      else if (i % dotOutputInterval == 0)
        os << "." << std::flush;
    percentageDone = until;
  }

  std::ostream& os; // The output stream the progress bar is printed to.

  int64_t numSteps;   // The number of steps that have to be done.
  int64_t stepsDone;  // The number of steps that have already been done.
  int percentageDone; // The percentage that has already been done.

  int percentageOutputInterval; // Percentage points between two printed percentages.
  int dotOutputInterval;        // Percentage points between two printed dots.
  bool verbose;                 // Indicates if the progress bar should be printed.
};
