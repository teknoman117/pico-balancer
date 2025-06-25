#ifndef UTILS_HPP_
#define UTILS_HPP

#include <utility>

#include "FreeRTOS.h"
#include "semphr.h"

class ScopedLock final {
    SemaphoreHandle_t mutex;

public:
    ScopedLock() = delete;
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;

    ScopedLock(SemaphoreHandle_t mutex_)
            : mutex(mutex_) {
        xSemaphoreTake(mutex, portMAX_DELAY);
    }

    ~ScopedLock() {
        if (mutex) {
            xSemaphoreGive(mutex);
        }
    }

    ScopedLock(ScopedLock&& lock_)
            : mutex(nullptr) {
        std::swap(lock_.mutex, mutex);
    }

    ScopedLock& operator=(ScopedLock&& lock_) {
        std::swap(lock_.mutex, mutex);
        return *this;
    }
};

template <typename T>
T normalize_heading_degrees(T heading) {
    if (heading > (T) 180) {
        return normalize_heading_degrees(heading - (T) 360);
    } else if (heading < (T) -180) {
        return normalize_heading_degrees(heading + (T) 360);
    } else {
        return heading;
    }
}

#endif /* UTILS_HPP */