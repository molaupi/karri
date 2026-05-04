#pragma once
#include <random>

struct ThreadSafeRandom {

    // Returns a random number between 0.0 and 1.0. Thread safe and with independent random number generators for each thread.
    static double randomNumber() {
        thread_local std::mt19937 generator(getSeed());
        std::uniform_real_distribution distribution{0.0, 1.0};
        return distribution(generator);
    }

private:

#ifdef KARRI_LOGIT_FIXED_SEED
    static constexpr uint32_t FIXED_SEED = KARRI_LOGIT_FIXED_SEED;
    static uint32_t getSeed() {
        return FIXED_SEED;
    }
#else
    static uint32_t getSeed() {
        return std::random_device{}();
    }

#endif
};
