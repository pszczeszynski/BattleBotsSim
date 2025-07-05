#pragma once
#include <iostream>
#include <cstring>
#include <string>

template <typename T>
class CircularDeque
{
private:
    T *arr;       // Circular array
    int capacity; // Capacity of the array
    int front;    // Index of the front element
    int rear;     // Index of the rear element
    int size;     // Current size of the deque

public:
    CircularDeque(int capacity)
    {
        this->capacity = capacity;
        arr = new T[capacity];
        front = 0;
        rear = -1;
        size = 0;
    }

    ~CircularDeque()
    {
        delete[] arr;
    }

    void push_back(const T& element)
    {
        rear = (rear + 1) % capacity;

        // Overwrite the oldest element if the deque is full
        if (isFull())
        {
            front = (front + 1) % capacity;
        }
        else
        {
            size++;
        }

        arr[rear] = element;
    }

    void pop_front(int n = 1)
    {
        if (isEmpty())
        {
            std::cout << "Deque is empty. Unable to pop elements.\n";
            return;
        }

        int elementsToRemove = n < size ? n : size; // min
        for (int i = 0; i < elementsToRemove; i++)
        {
            front = (front + 1) % capacity;
            size--;
        }
    }

    void pop_back(int n = 1)
    {
        if (isEmpty())
        {
            std::cout << "Deque is empty. Unable to pop elements.\n";
            return;
        }

        int elementsToRemove = n < size ? n : size; // min
        for (int i = 0; i < elementsToRemove; i++)
        {
            rear = (rear - 1 + capacity) % capacity;
            size--;
        }
    }

    void clear()
    {
        front = 0;
        rear = -1;
        size = 0;
    }

    bool isFull() const
    {
        return size == capacity;
    }

    bool isEmpty() const
    {
        return size == 0;
    }

    void display() const
    {
        if (isEmpty())
        {
            std::cout << "Deque is empty.\n";
            return;
        }

        std::cout << "Deque elements: ";
        int count = 0;
        int index = front;
        while (count < size)
        {
            std::cout << arr[index] << " ";
            index = (index + 1) % capacity;
            count++;
        }
        std::cout << std::endl;
    }

    /**
     * Returns the number of elements in the deque
    */
    int Size() const
    {
        return size;
    }

    /**
     * Copies the elements in the deque to the destination array
     * 
     * @param dest Destination array
     * @param start Index of the first element to copy
     * @param num_bytes Number of bytes to copy
     * @return the number of elements copied (or -1 if the number of elements to copy exceeds the size of the deque)
    */
    int copy_to(T *dest, int start, int num_bytes) const
    {
        // Calculating number of elements to copy based on bytes
        int num_elements = num_bytes / sizeof(T);
        // Check if the number of elements to copy exceeds the size of the deque
        if (num_elements > size || start + num_elements > size)
        {
            std::cerr << "ERROR copy_to: Not enough elements in the deque to copy." << std::endl;
            std::cerr << "  Number of elements to copy: " << num_elements << std::endl;
            std::cerr << "  Deque size: " << size << std::endl;
            // return failure
            return -1;
        }

        // Calculate real start index in the array
        int real_start = (front + start) % capacity;

        if (real_start + num_elements <= capacity)
        {
            // If there is no wrap around
            std::memcpy(dest, arr + real_start, num_bytes);
        }
        else
        {
            // If there is a wrap around
            int num_bytes_until_end = (capacity - real_start) * sizeof(T);
            int num_bytes_at_start = num_bytes - num_bytes_until_end;

            std::memcpy(dest, arr + real_start, num_bytes_until_end);
            std::memcpy(dest + (num_bytes_until_end / sizeof(T)), arr, num_bytes_at_start);
        }

        // return the number of elements copied
        return num_elements;
    }

    T &operator[](int index)
    {
        if (index < 0 || index >= size)
        {
            std::cerr << "Index is out of bounds." << std::endl;
            return arr[0];
        }

        return arr[(front + index) % capacity];
    }

    /**
     * Checks if the deque ends with the given sequence
    */
    bool endsWith(const std::string& seq) const
    {
        if (seq.length() > size) return false;

        for (int i = 0; i < seq.length(); ++i)
        {
            if (arr[(rear - i + capacity) % capacity] != seq[seq.length() - 1 - i])
                return false;
        }
        return true;
    }
};
