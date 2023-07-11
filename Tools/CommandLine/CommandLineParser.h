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

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>

#include "Tools/LexicalCast.h"

// A simple tool for obtaining command line options.
class CommandLineParser {
 public:
  // Constructs an uninitialized command line parser.
  CommandLineParser() = default;

  // Constructs a command line parser and parses the command line.
  CommandLineParser(int argc, char* argv[]) {
    parse(argc, argv);
  }

  // Parses the command line.
  void parse(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
      // Check if the current token is an option name.
      if (argv[i][0] != '-')
        throw std::invalid_argument("missing option name before '" + std::string(argv[i]) + "'");

      // The current option's name.
      std::string nm(&argv[i][1]);

      // Fetch the current option's value(s).
      options.erase(nm);
      for (; i + 1 < argc && argv[i + 1][0] != '-'; ++i)
        options.emplace(nm, argv[i + 1]);

      if (options.count(nm) == 0)
        options.emplace(nm, "");
    }
  }

  // Returns true if the specified option is set on the command line.
  bool isSet(const std::string& nm) const {
    return options.count(nm) > 0;
  }

  // Returns the (first) value of the specified option, or dflt if the option is not set.
  template <typename T>
  T getValue(const std::string& nm, const T& dflt = T()) const {
    return isSet(nm) ? lexicalCast<T>(options.lower_bound(nm)->second) : dflt;
  }

  // Returns a vector of all values of the specified option.
  template <typename T>
  std::vector<T> getValues(const std::string& nm) const {
    std::vector<T> values;
    for (auto iter = options.lower_bound(nm); iter != options.upper_bound(nm); ++iter)
      values.push_back(lexicalCast<T>(iter->second));
    return values;
  }

 private:
  std::multimap<std::string, std::string> options; // The command line options as name/value pairs.
};
