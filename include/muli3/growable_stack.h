#pragma once

#include "allocator.h"

namespace muli3
{

template <typename T, int32 N>
class GrowableStack
{
    static_assert(std::is_trivially_copyable_v<T> && std::is_trivially_destructible_v<T>);

public:
    GrowableStack()
        : array{ stackArray }
        , count{ 0 }
        , capacity{ N }
    {
    }

    GrowableStack(const GrowableStack&) = delete;
    GrowableStack& operator=(const GrowableStack&) = delete;

    GrowableStack(GrowableStack&& other) noexcept
    {
        MoveFrom(std::move(other));
    }

    GrowableStack& operator=(GrowableStack&& other) noexcept
    {
        if (this != &other)
        {
            reset();
            MoveFrom(std::move(other));
        }

        return *this;
    }

    ~GrowableStack()
    {
        if (array != stackArray)
        {
            muli3::Free(array);
        }
    }

    template <typename... Args>
    T& emplace_back(Args&&... args)
    {
        reserve(count + 1);

        T& slot = array[count++];
        slot = T{ std::forward<Args>(args)... };
        return slot;
    }

    void push_back(const T& v)
    {
        reserve(count + 1);
        array[count++] = v;
    }

    void push_back(T&& v)
    {
        reserve(count + 1);
        array[count++] = std::move(v);
    }

    T pop_back()
    {
        MuliAssert(count > 0);
        --count;
        return array[count];
    }

    T& back()
    {
        return array[count - 1];
    }

    const T& back() const
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

    void init()
    {
        array = stackArray;
        count = 0;
        capacity = N;
    }

    void reset()
    {
        if (array != stackArray)
        {
            muli3::Free(array);
        }
        init();
    }

    void clear()
    {
        count = 0;
    }

    void reserve(int32 newCapacity)
    {
        if (newCapacity <= capacity)
        {
            return;
        }

        T* old = array;
        int32 oldCount = count;
        capacity = std::max(newCapacity, capacity + capacity / 2);

        array = (T*)muli3::Alloc(capacity * sizeof(T));
        if (oldCount > 0)
        {
            memcpy(array, old, oldCount * sizeof(T));
        }

        if (old != stackArray)
        {
            muli3::Free(old);
        }
    }

    void resize(int32 newCount)
    {
        if (newCount <= count)
        {
            count = newCount;
            return;
        }

        reserve(newCount);
        memset(array + count, 0, (newCount - count) * sizeof(T));
        count = newCount;
    }

    void assign(int32 newCount, const T& value)
    {
        clear();
        reserve(newCount);

        while (count < newCount)
        {
            array[count++] = value;
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

    T& at(int32 index)
    {
        return array[index];
    }

    const T& at(int32 index) const
    {
        return array[index];
    }

    T& operator[](int32 index)
    {
        return array[index];
    }

    const T& operator[](int32 index) const
    {
        return array[index];
    }

private:
    void MoveFrom(GrowableStack&& other)
    {
        count = other.count;
        capacity = other.capacity;

        if (other.array == other.stackArray)
        {
            if (count > 0)
            {
                memcpy(stackArray, other.stackArray, count * sizeof(T));
            }
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