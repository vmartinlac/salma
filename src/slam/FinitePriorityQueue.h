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
        _weaker = 0;
        _stronger = 0;
    }

    void clear()
    {
        _items.clear();
        _priorities.clear();
        _max_size = 0;
        _size = 0;
        _weaker = 0;
        _stronger = 0;
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

                if( _size == 0 || priority < _priorities[_weaker] )
                {
                    _weaker = _size;
                }

                if( _size == 0 || priority > _priorities[_stronger] )
                {
                    _stronger = _size;
                }

                _size++;
            }
            else if( priority > _priorities[_weaker] )
            {
                _items[_weaker] = item;
                _priorities[_weaker] = priority;;

                _weaker = 0;
                _stronger = 0;

                for(size_t i=1; i<_size; i++)
                {
                    if( _priorities[i] < _priorities[_weaker] )
                    {
                        _weaker = i;
                    }
                    if( _priorities[i] > _priorities[_stronger] )
                    {
                        _stronger = i;
                    }
                }
            }
        }
    }

    priority_type top_priority()
    {
        if( _size == 0 )
        {
            throw std::runtime_error("internal error");
        }
        else
        {
            return _priorities[_stronger];
        }
    }

    item_type top()
    {
        if( _size == 0 )
        {
            throw std::runtime_error("internal error");
        }
        else
        {
            return _items[_stronger];
        }
    }

    void pop()
    {
        if( _size == 0 )
        {
            throw std::runtime_error("internal error");
        }
        else
        {
            if( _stronger != _size-1 )
            {
                std::swap(_items[_stronger], _items[_size-1]);
                std::swap(_priorities[_stronger], _priorities[_size-1]);
            }

            _size--;

            _weaker = 0;
            _stronger = 0;

            for(size_t i=1; i<_size; i++)
            {
                if( _priorities[i] < _priorities[_weaker] )
                {
                    _weaker = i;
                }
                if( _priorities[i] > _priorities[_stronger] )
                {
                    _stronger = i;
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
    size_t _stronger;
};

