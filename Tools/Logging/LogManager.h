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
#include <string>
#include <unordered_map>

#include "Tools/Logging/NullLogger.h"

// This class manages a set of named loggers. The logger type should be std::basic_ofstream or
// NullLogger. In the former case, each logger writes to a file whose name is the concatenation of a
// common base file name with the name of the logger. In the latter case, each logger discards the
// data written to it, avoiding any overhead at runtime. When a logger is requested, we return a
// reference to it if it already exists. Otherwise, the logger is newly constructed, an optional
// header line is written to it, and a reference to it is returned.
template <typename LoggerT = NullLogger>
class LogManager {
 public:
  LogManager() = delete;

  static LoggerT& getLogger(const std::string& name, const std::string& header = "") {
    auto iter = loggers.find(name);
    if (iter == loggers.end()) {
      iter = loggers.emplace(name, LoggerT(baseFileName + name)).first;
      LIGHT_KASSERT((bool) iter->second, "Could not open logger output stream at " << baseFileName + name << "!");
      iter->second << header;
    }
    return iter->second;
  }

  static void setBaseFileName(const std::string& fileName) {
    baseFileName = fileName;
  }

 private:
  inline static std::unordered_map<std::string, LoggerT> loggers;
  inline static std::string baseFileName;
};
