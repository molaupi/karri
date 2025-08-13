/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************


#pragma once
namespace karri {

    struct InputConfig {

    public:
        static InputConfig& getInstance()
        {
            static InputConfig instance; // Guaranteed to be destroyed.
            // Instantiated on first use.
            return instance;
        }

    private:
        InputConfig() = default;

    public:
        // public deleted constructors for compiler error messages
        InputConfig(InputConfig const&) = delete;
        void operator=(InputConfig const&) = delete;

        int maxWaitTime = -1;
        int stopTime = -1;
        int maxNumPickups = -1;
        int maxNumDropoffs = -1;
        double alpha = -1.0;
        int beta = -1;
        int requestBatchInterval = -1;
        double epsilon = -1.0;
        int phi = -1;
        bool alwaysUseVehicle = false;
    };

}