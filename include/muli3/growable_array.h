#pragma once

#include "allocator.h"

namespace muli3
{

template <typename T, int32 N>
class GrowableArray
{
public:
    GrowableArray()
        : array{ stackArray }
        , count{ 0 }
        , capacity{ N }
    {
    }

    GrowableArray(const GrowableArray&) = delete;
    GrowableArray& operator=(const GrowableArray&) = delete;

    GrowableArray(GrowableArray&& other) noexcept
        : array{ stackArray }
        , count{ 0 }
        , capacity{ N }
    {
        MoveFrom(std::move(other));
    }

    GrowableArray& operator=(GrowableArray&& other) noexcept
    {
        if (this != &other)
        {
            Release();
            array = stackArray;
            count = 0;
            capacity = N;
            MoveFrom(std::move(other));
        }

        return *this;
    }

    ~GrowableArray()
    {
        Release();
    }

    template <typename... Args>
    T& emplace_back(Args&&... args)
    {
        reserve(count + 1);

        return *new (array + count++) T{ std::forward<Args>(args)... };
    }

    T pop_back()
    {
        MuliAssert(count > 0);
        --count;
        return array[count];
    }

    T& back() const
    {
        return array[count - 1];
    }

    int32 size() const
    {
        return count;
    }

    int32 max_size() const
    {
        return capacity;
    }

    void clear()
    {
        count = 0;
    }

    void reset()
    {
        array = stackArray;
        count = 0;
        capacity = N;
    }

    void reserve(int32 newCapacity)
    {
        if (newCapacity <= capacity)
        {
            return;
        }

        T* old = array;
        int32 oldCount = count;
        capacity = (std::max)(newCapacity, capacity * 2);

        array = (T*)muli3::Alloc(capacity * sizeof(T));
        memcpy(array, old, oldCount * sizeof(T));

        if (old != stackArray)
        {
            muli3::Free(old);
        }
    }

    void resize(int32 newCount)
    {
        reserve(newCount);

        while (count < newCount)
        {
            emplace_back();
        }

        count = newCount;
    }

    void assign(int32 newCount, const T& value)
    {
        clear();
        reserve(newCount);

        while (count < newCount)
        {
            emplace_back(value);
        }
    }

    T* begin()
    {
        return array;
    }

    const T* begin() const
    {
        return array;
    }

    T* end()
    {
        return array + count;
    }

    const T* end() const
    {
        return array + count;
    }

    T* data()
    {
        return array;
    }

    const T* data() const
    {
        return array;
    }

    T& operator[](int32 index) const
    {
        return array[index];
    }

private:
    void Release()
    {
        if (array != stackArray)
        {
            muli3::Free(array);
            array = nullptr;
        }
    }

    void MoveFrom(GrowableArray&& other)
    {
        count = other.count;
        capacity = other.capacity;

        if (other.array == other.stackArray)
        {
            memcpy(stackArray, other.stackArray, count * sizeof(T));
            array = stackArray;
        }
        else
        {
            array = other.array;
            other.array = other.stackArray;
        }

        other.count = 0;
        other.capacity = N;
    }

    T* array;
    T stackArray[N];
    int32 count;
    int32 capacity;
};

} // namespace muli3
