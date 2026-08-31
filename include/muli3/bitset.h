#pragma once

#include "common.h"

namespace muli3
{

struct Bitset
{
    int32 groupCount;
    int32 wordCount;
    int32 wordStride;
    uint64* bits;
};

template <typename Alloc>
inline Bitset AllocateBitset(int32 bitCount, int32 groupCount, Alloc* alloc)
{
    int32 wordCount = (bitCount + 63) / 64;

    // Cache-line separated worker strides avoid false sharing
    int32 wordStride = (wordCount + 7) & ~7;
    int32 bitSize = groupCount * wordStride * sizeof(uint64);

    uint64* bits = (uint64*)alloc->Allocate(bitSize);
    std::memset(bits, 0, bitSize);

    return { groupCount, wordCount, wordStride, bits };
}

template <typename Alloc>
inline void FreeBitset(Bitset* bitset, Alloc* alloc)
{
    int32 bitSize = bitset->groupCount * bitset->wordStride * int32(sizeof(uint64));
    alloc->Free(bitset->bits, bitSize);
    bitset->bits = nullptr;
    bitset->groupCount = 0;
    bitset->wordCount = 0;
    bitset->wordStride = 0;
}

// Merge all groups' bits into the 0th group.
inline void MergeBitset(Bitset* bitset)
{
    uint64* groupBits = bitset->bits;
    for (int32 groupIndex = 1; groupIndex < bitset->groupCount; ++groupIndex)
    {
        const uint64* otherBits = bitset->bits + groupIndex * bitset->wordStride;
        for (int32 i = 0; i < bitset->wordCount; ++i)
        {
            groupBits[i] |= otherBits[i];
        }
    }
}

inline void SetBit(Bitset* bitset, int32 groupIndex, int32 bitIndex)
{
    uint64* groupBits = bitset->bits + groupIndex * bitset->wordStride;
    groupBits[bitIndex >> 6] |= uint64(1) << (bitIndex & 63);
}

inline bool GetBit(Bitset* bitset, int32 groupIndex, int32 bitIndex)
{
    uint64* groupBits = bitset->bits + groupIndex * bitset->wordStride;
    return (groupBits[bitIndex >> 6] & (uint64(1) << (bitIndex & 63))) != 0;
}

inline void ClearBit(Bitset* bitset, int32 groupIndex, int32 bitIndex)
{
    uint64* groupBits = bitset->bits + groupIndex * bitset->wordStride;
    groupBits[bitIndex >> 6] &= ~(uint64(1) << (bitIndex & 63));
}

inline void ToggleBit(Bitset* bitset, int32 groupIndex, int32 bitIndex)
{
    uint64* groupBits = bitset->bits + groupIndex * bitset->wordStride;
    groupBits[bitIndex >> 6] ^= uint64(1) << (bitIndex & 63);
}

} // namespace muli3
