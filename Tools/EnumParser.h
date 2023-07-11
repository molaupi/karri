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
#include <stdexcept>
#include <string>
#include <unordered_map>

// A facility for translating strings into enum values. Enums must specialize initNameToEnumMap.
template <typename T>
class EnumParser {
 public:
  // Constructs and initializes an enum parser.
  EnumParser() {
    initNameToEnumMap();
  }

  // Fills the map with name/value pairs. Enumerations must specialize this member function.
  void initNameToEnumMap() {
    assert(false);
  }

  // Returns the enum value with the specified name.
  T operator()(const std::string& name) const {
    const auto iter = nameToEnum.find(name);
    if (iter == nameToEnum.end())
      throw std::invalid_argument("no enum value with the specified name -- '" + name + "'");
    return iter->second;
  }

 private:
  std::unordered_map<std::string, T> nameToEnum; // A map to translate strings into enum values.
};
