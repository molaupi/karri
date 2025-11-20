#pragma once
#include <fstream>
#include <functional>
#include <vector>
#include <limits>
#include <random>
#include <bits/atomic_base.h>
#include <iostream>

#include "types.h"

namespace karri::mode_choice::utility_logit {
/**
 * A simple Logit utility distribution function. Keeps utilities and probabilities vector as reused state. Utility
 * calculation is defined in external function. Probabilities are calculated for each option i as e^(U_i) / Sum e^(U_j)
 * for all options in the choice set.
 *
 */
    template<typename T, typename P>
    class Logit {
        using UtilityFunction = std::function<double(const Attributes &, const P &)>;

    private:
        const std::vector<T> &options;
        mutable std::vector<double> utilities;
        mutable std::vector<double> expUtilities;
        mutable std::vector<double> probabilities;
        UtilityFunction utilityFunction;
        std::unordered_map<T, P> paramsMap;

        void flush() const {
            for (size_t i = 0; i < options.size(); ++i) {
                utilities[i] = -std::numeric_limits<double>::infinity();
                expUtilities[i] = -std::numeric_limits<double>::infinity();
                probabilities[i] = 0.0;
            }
        }

        static double randomNumber() {
            thread_local std::mt19937 generator{std::random_device{}()};
            std::uniform_real_distribution distribution{0.0, 1.0};
            return distribution(generator);
        }

    public:
        explicit Logit(UtilityFunction utility, const std::vector<T> &options,
                       std::unordered_map<T, P> params) : options(options), utilities(options.size(),
                                                                                      -std::numeric_limits<double>::infinity()),
                                                          expUtilities(options.size(),
                                                                       -std::numeric_limits<double>::infinity()),
                                                          probabilities(options.size(), 0),
                                                          utilityFunction(std::move(utility)),
                                                          paramsMap(std::move(params)) {
        }

        T select(const std::vector<Alternative<T> > &elements) const {
            KASSERT(elements.size() == options.size());
            flush();

            double sum = 0.0;
            for (size_t i = 0; i < elements.size(); ++i) {
                if (!elements[i].enabled) {
                    expUtilities[i] = 0.0;
                    continue;
                }
                const double utility = utilityFunction(elements[i].data, paramsMap.at(elements[i].option));
                utilities[i] = utility;
                const double expUtility = exp(utility);
                expUtilities[i] = expUtility;
                sum += expUtility;
            }
            double cumulativeSum = 0.0;
            const double rnd = randomNumber();
            T selectedElement = elements[0].option;

            for (size_t i = 0; i < elements.size(); ++i) {
                const double probability = expUtilities[i] / sum;
                probabilities[i] = probability;
                if (cumulativeSum < rnd) {
                    selectedElement = elements[i].option;
                }
                cumulativeSum += probability;
            }

            return selectedElement;
        }

        void printCurrentState() const {
            std::cout << "Utilities:\n";
            for (const double u: utilities) std::cout << "  " << u << "\n";
            std::cout << "Exp.-Utilities:\n";
            for (const double u: expUtilities) std::cout << "  " << u << "\n";

            std::cout << "Probabilities:\n";
            for (const double p: probabilities) std::cout << "  " << p << "\n";
        }
    };
}