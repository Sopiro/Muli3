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

    ~GrowableArray()
    {
        if (array != stackArray)
        {
            muli3::Free(array);
            array = nullptr;
        }
    }

    template <typename... Args>
    T& EmplaceBack(Args&&... args)
    {
        if (count == capacity)
        {
            T* old = array;
            capacity *= 2;

            array = (T*)muli3::Alloc(capacity * sizeof(T));
            memcpy(array, old, count * sizeof(T));

            if (old != stackArray)
            {
                muli3::Free(old);
            }
        }

        return *new (array + count++) T{ std::forward<Args>(args)... };
    }

    T PopBack()
    {
        MuliAssert(count > 0);
        --count;
        return array[count];
    }

    T& Back() const
    {
        return array[count - 1];
    }

    int32 Count() const
    {
        return count;
    }

    void Clear()
    {
        count = 0;
    }

    T& operator[](int32 index) const
    {
        return array[index];
    }

private:
    T* array;
    T stackArray[N];
    int32 count;
    int32 capacity;
};

} // namespace muli3

