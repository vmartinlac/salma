#pragma once

#include <QMutex>
#include <QObject>

class PortBase : public QObject
{
    Q_OBJECT

public:

    PortBase(QObject* parent=nullptr) : QObject(parent) { }

signals:

    void updated();
};

template<typename T>
class Port : public PortBase
{
public:

    Port(QObject* parent=nullptr) : PortBase(parent) { }

    void beginWrite();
    void endWrite();

    void beginRead();
    void endRead();

    void read(T& to);
    void write(const T& from);

    T& data();

protected:

    T mData;
    QMutex mMutex;
};

template<typename T>
T& Port<T>::data()
{
    return mData;
}

template<typename T>
void Port<T>::beginWrite()
{
    mMutex.lock();
}

template<typename T>
void Port<T>::endWrite()
{
    mMutex.unlock();
    updated();
}

template<typename T>
void Port<T>::beginRead()
{
    mMutex.lock();
}

template<typename T>
void Port<T>::endRead()
{
    mMutex.unlock();
}

template<typename T>
void Port<T>::read(T& to)
{
    beginRead();
    to = mData;
    endRead();
}

template<typename T>
void Port<T>::write(const T& from)
{
    beginWrite();
    mData = from;
    endWrite();
}

