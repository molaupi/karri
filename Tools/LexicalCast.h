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

#include <stdexcept>
#include <string>
#include <type_traits>

#include <csv.h>

#include "Tools/StringHelpers.h"

// Converts the specified string to type T.
template <typename T, typename StringT, typename = std::enable_if_t<!std::is_enum<T>::value>>
inline T lexicalCast(const StringT& string) {
  try {
    T val;
    io::detail::parse<io::throw_on_overflow>(const_cast<char*>(str::cStr(string)), val);
    return val;
  } catch (io::error::invalid_single_character& e) {
    const auto what = "'" + std::string(string) + "' is not a single character";
    throw std::invalid_argument(what);
  } catch (io::error::no_digit& e) {
    const auto what = "'" + std::string(string) + "' cannot be converted to an arithmetic type";
    throw std::invalid_argument(what);
  } catch (io::error::integer_overflow& e) {
    const auto what = "'" + std::string(string) + "' is outside the range of representable values";
    throw std::out_of_range(what);
  } catch (io::error::integer_underflow& e) {
    const auto what = "'" + std::string(string) + "' is outside the range of representable values";
    throw std::out_of_range(what);
  }
}

// Converts the specified string to the specified enum type.
template <typename EnumT, typename StringT>
inline std::enable_if_t<std::is_enum<EnumT>::value, EnumT> lexicalCast(const StringT& string) {
  return static_cast<EnumT>(lexicalCast<std::underlying_type_t<EnumT>>(string));
}
