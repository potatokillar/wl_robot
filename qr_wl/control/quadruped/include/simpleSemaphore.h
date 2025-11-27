/*! @file SharedMemory.h
 *  @brief Shared memory utilities for connecting the simulator program to the
 * robot program
 *
 *
 */
#ifndef SIMPLESEMAPHORE_H
#define SIMPLESEMAPHORE_H

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cassert>
#include <cstring>
#include <stdexcept>
#include <string>

// #include "cTypes.h"

#define DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME "development-simulator"

/*!
 * A POSIX semaphore for shared memory.
 * See https://linux.die.net/man/7/sem_overview for more deatils
 */
class SimpleSemaphore
{
public:
    /*!
     * If semaphore is unitialized, initialize it and set its value.  This can be
     * called as many times as you want safely. It must be called at least once.
     * Only one process needs to call this, even if it is used in multiple
     * processes.
     *
     * Note that if init() is called after the semaphore has been initialized, it
     * will not change its value.
     * @param value The initial value of the semaphore.
     */
    void init(unsigned int value)
    {
        if (!_init) {
            if (sem_init(&_sem, 1, value)) {
                printf("[ERROR] Failed to initialize shared memory semaphore: %s\n",
                       strerror(errno));
            } else {
                _init = true;
            }
        }
    }

    ~SimpleSemaphore()
    {
        destroy();
    }

    /*!
     * Increment the value of the semaphore.
     */
    void increment() { sem_post(&_sem); }

    /*!
     * If the semaphore's value is > 0, decrement the value.
     * Otherwise, wait until its value is > 0, then decrement.
     */
    void decrement() { sem_wait(&_sem); }

    /*!
     * If the semaphore's value is > 0, decrement the value and return true
     * Otherwise, return false (doesn't decrement or wait)
     * @return
     */
    bool tryDecrement() { return (sem_trywait(&_sem)) == 0; }

    /*!
     * Like decrement, but after waiting ms milliseconds, will give up
     * Returns true if the semaphore is successfully decremented
     */
    bool decrementTimeout(uint64_t seconds, uint64_t nanoseconds)
    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += nanoseconds;
        ts.tv_sec += seconds;
        ts.tv_sec += ts.tv_nsec / 1000000000;
        ts.tv_nsec %= 1000000000;
#ifdef __linux__
        return (sem_timedwait(&_sem, &ts) == 0);
#else
        return (sem_trywait(&_sem) == 0);
#endif
    }

    /*!
     * Delete the semaphore.  Note that deleting a semaphore in one process while
     * another is still using it results in very strange behavior.
     */
    void destroy() { sem_destroy(&_sem); }

private:
    sem_t _sem;
    bool _init = false;
};

#endif