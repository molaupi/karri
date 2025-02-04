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
#include <cstring>
#include <fstream>
#include <string>
#include <type_traits>
#include <vector>
#include "DataStructures/Containers/BitVector.h"

namespace bio {

// Returns the number of bytes the specified self-contained object occupies on disk.
template <typename T>
inline int size(const T& /*obj*/) {
  return sizeof(T);
}

// Returns the number of bytes the specified string occupies on disk.
inline int size(const std::string& str) {
  return str.size() + 1;
}

// Returns the number of bytes the specified C-style string occupies on disk.
inline int size(const char* const str) {
  return std::strlen(str) + 1;
}

// Returns the number of bytes the specified vector of self-contained objects occupies on disk.
template <typename T, typename AllocT>
inline std::enable_if_t<std::is_trivially_copyable<T>::value, int>
size(const std::vector<T, AllocT>& vec) {
  return sizeof(int) + vec.size() * sizeof(T);
}

// Returns the number of bytes the specified vector occupies on disk.
template <typename T, typename AllocT>
inline std::enable_if_t<!std::is_trivially_copyable<T>::value, int>
size(const std::vector<T, AllocT>& vec) {
  int s = sizeof(int);
  for (const auto& elem : vec)
    s += size(elem);
  return s;
}

// Returns the number of bytes the specified bit-vector occupies on disk.
inline int size(const BitVector& vec) {
  return 2 * sizeof(int) + vec.numBlocks() * sizeof(typename BitVector::Block);
}

// Reads a self-contained object from a binary file.
template <typename T>
inline void read(std::ifstream& in, T& obj) {
  in.read(reinterpret_cast<char*>(&obj), sizeof(T));
  assert(in.good());
}

// Reads a string from a binary file.
inline void read(std::ifstream& in, std::string& str) {
  getline(in, str, '\0');
  assert(in.good());
}

// Reads a vector of self-contained objects from a binary file.
template <typename T, typename AllocT>
inline std::enable_if_t<std::is_trivially_copyable<T>::value>
read(std::ifstream& in, std::vector<T, AllocT>& vec) {
  int size;
  read(in, size);
  vec.resize(size);
  in.read(reinterpret_cast<char*>(vec.data()), size * sizeof(T));
  assert(in.good());
}

// Reads a vector of objects from a binary file.
template <typename T, typename AllocT>
inline std::enable_if_t<!std::is_trivially_copyable<T>::value>
read(std::ifstream& in, std::vector<T, AllocT>& vec) {
  int size;
  read(in, size);
  vec.resize(size);
  for (auto& elem : vec)
    read(in, elem);
  assert(in.good());
}

// Reads a bit-vector from a binary file.
inline void read(std::ifstream& in, BitVector& vec) {
  int numBits;
  read(in, numBits);
  std::vector<typename BitVector::Block> blocks;
  read(in, blocks);
  vec = BitVector(blocks.begin(), blocks.end(), numBits);
}

// Writes a self-contained object to a binary file.
template <typename T>
inline void write(std::ofstream& out, const T& obj) {
  out.write(reinterpret_cast<const char*>(&obj), sizeof(T));
  assert(out.good());
}

// Writes a string to a binary file.
inline void write(std::ofstream& out, const std::string& str) {
  out.write(str.data(), str.size() + 1);
  assert(out.good());
}

// Writes a C-style string to a binary file.
inline void write(std::ofstream& out, const char* const str) {
  out.write(str, std::strlen(str) + 1);
  assert(out.good());
}

// Writes a vector of self-contained objects to a binary file.
template <typename T, typename AllocT>
inline std::enable_if_t<std::is_trivially_copyable<T>::value>
write(std::ofstream& out, const std::vector<T, AllocT>& vec) {
  write(out, static_cast<int>(vec.size()));
  out.write(reinterpret_cast<const char*>(vec.data()), vec.size() * sizeof(T));
  assert(out.good());
}

// Writes a vector of objects to a binary file.
template <typename T, typename AllocT>
inline std::enable_if_t<!std::is_trivially_copyable<T>::value>
write(std::ofstream& out, const std::vector<T, AllocT>& vec) {
  write(out, static_cast<int>(vec.size()));
  for (const auto& elem : vec)
    write(out, elem);
  assert(out.good());
}

// Writes a bit-vector to a binary file.
inline void write(std::ofstream& out, const BitVector& vec) {
  write(out, static_cast<int>(vec.size()));
  write(out, vec.getBlocks());
}

}
