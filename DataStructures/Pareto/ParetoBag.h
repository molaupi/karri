#pragma once
#include <concepts>
#include <vector>

// A type that can be stored in a ParetoBag: It is copy-constructible and provides a static method
// bool dominates(const T&, const T&) that decides whether the first entry dominates the second.
template<typename T>
concept ParetoBagEntry = std::copy_constructible<T> && requires(const T &a, const T &b) {
    { T::dominates(a, b) } -> std::same_as<bool>;
};

// A bag of mutually non-dominated entries.
template<ParetoBagEntry T>
struct ParetoBag {

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

    // Adds the candidate to the bag unless it is dominated by an entry in the bag. Removes all entries that are
    // dominated by the candidate. Returns true if the candidate was added.
    bool addCandidate(const T &candidate) {
        if (isDominated(candidate))
            return false;
        addNonDominatedCandidate(candidate);
        return true;
    }

    // Returns true if the candidate is dominated by any of the entries in the bag.
    bool isDominated(const T &candidate) const {
        for (const auto &existing: entries) {
            if (T::dominates(existing, candidate))
                return true;
        }
        return false;
    }

    // Adds a candidate that is known not to be dominated by any entry in the bag (see isDominated()). Removes all
    // entries that are dominated by the candidate.
    void addNonDominatedCandidate(const T &candidate) {
        // Remove all existing entries that are dominated by the new entry (without maintaining order):
        int k = 0;
        while (k < entries.size()) {
            if (T::dominates(candidate, entries[k])) {
                entries[k] = entries.back();
                entries.pop_back();
                continue;
            }
            ++k;
        }

        // Add the new entry
        entries.push_back(candidate);
    }

};
