#ifndef PAIR_HASH_H
#define PAIR_HASH_H

#include <functional>
#include <utility> // For std::pair

// Define the custom hash function for std::pair<int, int>
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator ()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);  // Combine the two hashes using bitwise operations
    }
};

#endif // PAIR_HASH_H
