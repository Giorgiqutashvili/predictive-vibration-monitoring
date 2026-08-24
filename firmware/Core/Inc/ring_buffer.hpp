/*
 * ring_buffer.hpp
 *
 *  Created on: Aug 24, 2026
 *      Author: lukam
 */

#ifndef INC_RING_BUFFER_HPP_
#define INC_RING_BUFFER_HPP_

#pragma once

#include <cstdint>
#include <cstddef>

// Struct representing a single 3-axis vibration sample
struct VibrationSample {
    float x;
    float y;
    float z;
};

// Fixed-size circular ring buffer template
template <size_t CAPACITY>
class CircularBuffer {
private:
    VibrationSample buffer[CAPACITY];
    size_t head = 0;
    size_t tail = 0;
    size_t count = 0;

public:
    // Push a new sample into the buffer (overwrites oldest data if full)
    void push(const VibrationSample &sample) {
        buffer[head] = sample;
        head = (head + 1) % CAPACITY;

        if (count < CAPACITY) {
            count++;
        } else {
            // Buffer full: advance tail to discard oldest sample
            tail = (tail + 1) % CAPACITY;
        }
    }

    // Pop the oldest sample from the buffer
    bool pop(VibrationSample &sample) {
        if (isEmpty()) {
            return false;
        }
        sample = buffer[tail];
        tail = (tail + 1) % CAPACITY;
        count--;
        return true;
    }

    // Status helper methods
    bool isFull() const { return count == CAPACITY; }
    bool isEmpty() const { return count == 0; }
    size_t size() const { return count; }
    size_t capacity() const { return CAPACITY; }

    void clear() {
        head = 0;
        tail = 0;
        count = 0;
    }
};



#endif /* INC_RING_BUFFER_HPP_ */
