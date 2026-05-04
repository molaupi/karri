/*******************************************************************************
 * MIT License
 *
 * This file is part of Mt-KaHyPar.
 *
 * Copyright (C) 2019 Lars Gottesbüren <lars.gottesbueren@kit.edu>
 * Copyright (C) 2019 Tobias Heuer <tobias.heuer@kit.edu>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#pragma once

#include <unordered_map>
#include <thread>
#include <mutex>
#include <sstream>
#include <assert.h>
#include <iostream>

#include <sched.h>

#define THREAD_ID sched_getcpu()

#include "tbb/task_scheduler_observer.h"

namespace parallel {

    template<typename HwTopology>
    class ThreadPinningObserver : public tbb::task_scheduler_observer {
        using Base = tbb::task_scheduler_observer;

    public:

        // Observer is pinned to the global task arena and is responsible for
        // pinning threads to unique CPU id.
        explicit ThreadPinningObserver(const std::vector<int> &cpus) :
                Base(),
                _num_cpus(HwTopology::instance().num_cpus()),
                _cpus(cpus) {
            if (_cpus.size() == 1) {
                _cpus.push_back(HwTopology::instance().get_backup_cpu(0, _cpus[0]));
            }

            observe(true); // Enable thread pinning
        }


        ThreadPinningObserver(const ThreadPinningObserver &) = delete;

        ThreadPinningObserver &operator=(const ThreadPinningObserver &) = delete;

        ThreadPinningObserver(ThreadPinningObserver &&other) :
                _num_cpus(other._num_cpus),
                _cpus(other._cpus) {}

        ThreadPinningObserver &operator=(ThreadPinningObserver &&) = delete;

        void on_scheduler_entry(bool) override {
            const int slot = tbb::this_task_arena::current_thread_index();
            assert(static_cast<size_t>(slot) < _cpus.size());

            if (slot >= static_cast<int>(_cpus.size())) {
                std::stringstream thread_id;
                thread_id << std::this_thread::get_id();
                throw std::invalid_argument(
                        "Thread " + thread_id.str() + " entered the global task arena "
                                                      "in a slot that should not exist (Slot = " +
                        std::to_string(slot) + ", Max Slots = " + std::to_string(_cpus.size()) +
                        ", slots are 0-indexed). This bug only occurs in older versions of TBB. "
                        "We recommend upgrading TBB to the newest version.");
            }

//            std::cout << pin_thread_message(_cpus[slot]);
            pin_thread_to_cpu(_cpus[slot]);
        }

        void on_scheduler_exit(bool) override {
//            std::cout << "Thread with PID" << std::this_thread::get_id()
//                      << "leaves GLOBAL task arena";
        }

    private:

        void pin_thread_to_cpu(const int cpu_id) {
#if __linux__
            const size_t size = CPU_ALLOC_SIZE(_num_cpus);
            cpu_set_t mask;
            CPU_ZERO(&mask);
            CPU_SET(cpu_id, &mask);
            const int err = sched_setaffinity(0, size, &mask);

            if (err) {
                const int error = errno;
                throw std::runtime_error(
                        "Failed to set thread affinity to cpu" + std::to_string(cpu_id) + "." + std::to_string(error));
            }

            assert(THREAD_ID == cpu_id);
//            std::cout << "Thread with PID" << std::this_thread::get_id()
//                      << "successfully pinned to CPU" << cpu_id;
#endif
        }

        std::string pin_thread_message(const int cpu_id) {
            std::stringstream ss;
            ss << "Assign thread with PID " << std::this_thread::get_id()
               << " to CPU " << cpu_id << " in GLOBAL task arena";

            return ss.str();
        }

        std::string unpin_thread_message() {
            std::stringstream ss;
            ss << "Unassign thread with PID " << std::this_thread::get_id()
               << " on CPU " << THREAD_ID << " from GLOBAL task arena";

            return ss.str();
        }

        const int _num_cpus;
        std::vector<int> _cpus;
    };
}  // namespace parallel
