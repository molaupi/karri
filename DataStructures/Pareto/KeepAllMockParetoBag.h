#pragma once
#include <concepts>
#include <vector>


// A container that mocks the interface of a ParetoBag, but simply stores all candidates passed to it.
template<typename T>
struct KeepAllMockParetoBag {

    std::vector<T> entries;

    auto begin() const {
        return entries.begin();
    }

    auto end() const {
        return entries.end();
    }

    void clear() {
        entries.clear();
    }

    bool addCandidate(const T &candidate) {
        entries.push_back(candidate);
        return true;
    }

    bool isDominated(const T &) const {
        return false;
    }

    void addNonDominatedCandidate(const T &candidate) {
        entries.push_back(candidate);
    }

};
