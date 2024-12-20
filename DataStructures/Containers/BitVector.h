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
#include <limits>
#include <vector>

#include "Tools/Simd/AlignedVector.h"
#include "Tools/Bitwise.h"
#include "Tools/custom_assertion_levels.h"

// This class implements a vector of bits. The bits are divided into 64-bit blocks. Different bits
// can be modified concurrently when they are in different blocks. The first block is aligned on a
// cache line boundary.
class BitVector {
public:
    using Block = uint64_t; // The unsigned integer type in which the bits are stored.

    // A class that simulates the behavior of references to a single bit in a bit vector.
    class Reference {
        friend BitVector; // Only bit vectors are allowed to construct a reference.

    public:
        // Returns the value of the bit.
        operator bool() const {
            return getBit(block, bitIndex);
        }

        // Sets the bit to bitValue.
        Reference &operator=(const bool bitValue) {
            setBit(block, bitIndex, bitValue);
            return *this;
        }

        // Sets the bit to the value of the specified bit.
        Reference &operator=(const Reference &other) {
            return *this = static_cast<bool>(other);
        }

    private:
        // Constructs a reference to the bit with the specified index in the specified block.
        Reference(Block &block, const int bitIndex) : block(block), bitIndex(bitIndex) {
            assert(bitIndex >= 0);
            assert(bitIndex < std::numeric_limits<Block>::digits);
        }

        Block &block;       // The block in which the bit is stored.
        const int bitIndex; // The index of the bit within its block.
    };

    // The number of bits in a block.
    static constexpr int BITS_PER_BLOCK = std::numeric_limits<Block>::digits;

    // Constructs a bit vector of the specified size. All bits are initialized to init.
    explicit BitVector(const int size = 0, const bool init = false) : numBits(0) {
        resize(size, init);
    }

    // Constructs a bit vector containing the given blocks of bits.
    template<typename BlockIt>
    explicit BitVector(BlockIt begin, BlockIt end, const int numBits) : blocks(begin, end), numBits(numBits) {
        LIGHT_KASSERT(numBits > (end - begin - 1) * BITS_PER_BLOCK && numBits <= (end - begin) * BITS_PER_BLOCK);
    }

    // Returns the number of bits in this bit vector.
    int size() const {
        return numBits;
    }

    // Returns the number of blocks in this bit vector.
    int numBlocks() const {
        return blocks.size();
    }

    // Returns the number of bits set to true in this bit vector.
    int cardinality() const {
        auto cardinality = 0;
        for (const auto block: blocks)
            cardinality += bitCount(block);
        return cardinality;
    }

    // Changes the number of bits in this bit vector. Newly inserted bits are initialized to init.
    void resize(const int size, const bool init = false) {

        if (size == 0) {
            numBits = 0;
            blocks.clear();
            return;
        }

        const auto numUsedBits = numBits % BITS_PER_BLOCK;
        if (init && numUsedBits > 0)
            blocks.back() |= static_cast<Block>(-1) << numUsedBits;
        blocks.resize((size + BITS_PER_BLOCK - 1) / BITS_PER_BLOCK, init ? -1 : 0);
        const auto numUnusedBits = (BITS_PER_BLOCK - size % BITS_PER_BLOCK) % BITS_PER_BLOCK;
        blocks.back() &= static_cast<Block>(-1) >> numUnusedBits;
        numBits = size;
    }

    // Returns the bit with the specified index.
    bool operator[](const int bitIndex) const {
        assert(bitIndex >= 0);
        assert(bitIndex < size());
        return getBit(blocks[bitIndex / BITS_PER_BLOCK], bitIndex % BITS_PER_BLOCK);
    }

    // Returns a reference to the bit with the specified index.
    Reference operator[](const int bitIndex) {
        assert(bitIndex >= 0);
        assert(bitIndex < size());
        return {blocks[bitIndex / BITS_PER_BLOCK], bitIndex % BITS_PER_BLOCK};
    }

    // Returns the block with the specified index.
    Block block(const int blockIndex) const {
        assert(blockIndex >= 0);
        assert(blockIndex < numBlocks());
        return blocks[blockIndex];
    }

    const AlignedVector<Block>& getBlocks() const {
        return blocks;
    }

    // Returns the index of the first one-bit. If no such bit exists then -1 is returned.
    int firstSetBit() const {
        int blockIndex = 0;
        while (blockIndex < blocks.size() && blocks[blockIndex] == 0) ++blockIndex;
        if (blockIndex == blocks.size()) return -1;
        return blockIndex * BITS_PER_BLOCK + numTrailingZeros(blocks[blockIndex]);
    }

    // Returns the index of the first one-bit that occurs after fromIndex.
    // If no such bit exists then -1 is returned.
    int nextSetBit(int fromIndex) const {
        assert(fromIndex >= 0);
        assert(fromIndex < size());
        if (++fromIndex == size()) return -1;
        int blockIndex = fromIndex / BITS_PER_BLOCK;
        const auto firstBlock = blocks[blockIndex] >> fromIndex % BITS_PER_BLOCK;
        if (firstBlock != 0) return fromIndex + numTrailingZeros(firstBlock);
        ++blockIndex;
        while (blocks[blockIndex] == 0 && blockIndex < blocks.size()) ++blockIndex;
        if (blockIndex == blocks.size()) return -1;
        return blockIndex * BITS_PER_BLOCK + numTrailingZeros(blocks[blockIndex]);
    }

    // Inverts every bit in the BitVector
    void flip() {
        for (auto &b: blocks)
            b = ~b;

        // Make sure the unused bits in the last block are set to 0
        const auto numUnusedBits = (BITS_PER_BLOCK - numBits % BITS_PER_BLOCK) % BITS_PER_BLOCK;
        blocks.back() &= static_cast<Block>(-1) >> numUnusedBits;
    }

private:
    AlignedVector<Block> blocks; // The blocks in which the bits are stored.
    int numBits;                 // The number of bits in this bit vector.
};
