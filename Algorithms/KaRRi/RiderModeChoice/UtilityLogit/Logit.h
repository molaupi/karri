#pragma once
#include <fstream>
#include <functional>
#include <vector>
#include <limits>
#include <random>
#include <bits/atomic_base.h>
#include <iostream>

#include "types.h"
#include "Tools/ThreadSafeRandom.h"

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
        mutable std::vector<double> utilities;
        mutable std::vector<double> expUtilities;
        mutable std::vector<double> probabilities;
        UtilityFunction utilityFunction;
        std::unordered_map<T, P> paramsMap;

        void flush() const {
            for (auto &u: utilities)
                u = -std::numeric_limits<double>::infinity();
            for (auto &eu: expUtilities)
                eu = -std::numeric_limits<double>::infinity();
            for (auto &p: probabilities)
                p = 0.0;
        }

    public:
        explicit Logit(UtilityFunction utility, std::unordered_map<T, P> params)
            : utilities(params.size(), -std::numeric_limits<double>::infinity()),
              expUtilities(params.size(), -std::numeric_limits<double>::infinity()),
              probabilities(params.size(), 0),
              utilityFunction(std::move(utility)),
              paramsMap(std::move(params)) {
        }

        T select(const std::vector<Alternative<T> > &elements) const {
            flush();

            double sum = 0.0;
            for (size_t i = 0; i < elements.size(); ++i) {
                const double utility = utilityFunction(elements[i].data, paramsMap.at(elements[i].option));
                utilities[i] = utility;
                const double expUtility = exp(utility);
                expUtilities[i] = expUtility;
                sum += expUtility;
            }
            double cumulativeSum = 0.0;
            const double rnd = ThreadSafeRandom::randomNumber();
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
