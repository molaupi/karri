/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

        struct EllipseReconstructorStats {
            int numVerticesSettled = 0;
            int numEdgesRelaxed = 0;
            int64_t initTime = 0;
            int64_t topoSearchTime = 0;
            int64_t postprocessTime = 0;

            void reset() {
                numVerticesSettled = 0;
                numEdgesRelaxed = 0;
                initTime = 0;
                topoSearchTime = 0;
                postprocessTime = 0;
            }

            EllipseReconstructorStats& operator+=(const EllipseReconstructorStats& other) {
                numVerticesSettled += other.numVerticesSettled;
                numEdgesRelaxed += other.numEdgesRelaxed;
                initTime += other.initTime;
                topoSearchTime += other.topoSearchTime;
                postprocessTime += other.postprocessTime;
                return *this;
            }
        };


} // karri
