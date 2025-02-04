/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
/// Copyright (c) 2025 Moritz Laupichler
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

#include <string>
#include <vector>
#include "Tools/ContainerHelpers.h"
#include "Tools/BinaryIO.h"

class OnlyAttributesAsBinaryFilesExporter {
public:
    OnlyAttributesAsBinaryFilesExporter(bool = false/* compress */) {}

    void setBaseOutName(const std::string &newBaseOutName) {
        baseOutName = newBaseOutName;
    }

    void ignoreAttribute(const std::string &attrName) {
        attrsToIgnore.push_back(attrName);
    }

    template<typename OutEdgeRangesT, typename EdgeHeadsT>
    void writeTopology(const OutEdgeRangesT &, const EdgeHeadsT &) {
        // no op
    }


    // Overload for POD attribute types
    template<typename AttrType,
            typename AllocT>
    void
    writeAttribute(const std::vector<AttrType, AllocT> &values, const std::string &attrName) {
        if (contains(attrsToIgnore.begin(), attrsToIgnore.end(), attrName))
            return;
        const auto outName = baseOutName + "." + attrName + ".bin";
        std::ofstream out(outName);
        if (!out.good())
            throw std::invalid_argument("attribute output file cannot be opened -- '" + outName + "'");
        bio::write(out, values);
    }

private:

    std::string baseOutName;
    std::vector<std::string> attrsToIgnore;
};
