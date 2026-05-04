#pragma once
#include <fstream>
#include <functional>
#include <iostream>
#include <vector>
#include <random>
#include <sstream>

#include "types.h"

namespace karri::mode_choice::utility_logit {

    template<typename T, typename P>
    std::unordered_map<T, P> loadParametersFromCSV(
            std::basic_stringstream<char> file,
            std::function<T(const std::string &)> keyConverter,
            std::function<P(const ParameterMap &)> parameterConverter) {
        std::unordered_map<T, P> params;
//        std::ifstream file(filename);
        std::string line;

        // Read header
        while (line.empty()) {
            if (!std::getline(file, line)) {
                throw std::runtime_error("Failed to read CSV header");
            }
        }
        std::stringstream headerStream(line);
        std::string headerCell;

        // Skip the first header cell (key column)
        std::getline(headerStream, headerCell, ',');

        std::vector<std::string> paramNames;
        while (std::getline(headerStream, headerCell, ',')) {
            paramNames.push_back(headerCell);
        }

        // Read data lines
        while (std::getline(file, line)) {

            // Skip empty lines
            if (line.empty()) continue;

            std::stringstream ss(line);
            std::string keyStr;
            std::getline(ss, keyStr, ',');

            ParameterMap mp;
            for (const auto &paramName: paramNames) {
                std::string valStr;
                if (!std::getline(ss, valStr, ',')) {
                    throw std::runtime_error("Mismatch between header and data columns");
                }
                mp.values[paramName] = std::stod(valStr); // convert string to double here
            }

            T key = keyConverter(keyStr);
            params[key] = parameterConverter(std::move(mp));
        }

        return params;
    }
}