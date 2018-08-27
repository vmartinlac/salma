#pragma once

#include <vector>

template<typename item_type, typename priority_type>
class FinitePriorityQueue
{
public:

    FinitePriorityQueue()
    {
        clear();
    }

    FinitePriorityQueue(size_t max_size)
    {
        reset(max_size);
    }

    void reset(size_t max_size)
    {
        _items.resize( max_size );
        _priorities.resize( max_size );
        _max_size = max_size;
        _size = 0;
    }

    void clear()
    {
        _size = 0;
        _max_size = 0;
        _items.clear();
        _priorities.clear();
    }

    size_t size() { return _size; }

    size_t max_size() { return _max_size; }

    void push(const item_type& item, const priority_type& priority)
    {
        if( _max_size > 0 )
        {
            if( _size < _max_size )
            {
                _items[_size] = item;
                _priorities[_size] = priority;
                if( _size == 0 || priority < _priorities[_weaker])
                {
                    _weaker = _size;
                }
                _size++;
            }
            else if( priority > _priorities[_weaker] )
            {
                _items[_weaker] = item;
                _priorities[_weaker] = priority;;

                _weaker = 0;
                for(size_t i=1; i<_size; i++)
                {
                    if(_priorities[i] < _priorities[_weaker])
                    {
                        _weaker = i;
                    }
                }
            }
        }
    }

    item_type* begin()
    {
        return &_items.front();
    }

    item_type* end()
    {
        return &_items.front() + _size;
    }

protected:

    std::vector<item_type> _items;
    std::vector<priority_type> _priorities;
    size_t _size;
    size_t _max_size;
    size_t _weaker;
};

