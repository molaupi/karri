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

#include <nlohmann/json.hpp>
#include "MobitoppErrors.h"
#include "Tools/StringHelpers.h"

static nlohmann::json parseMobitoppJsonMsg(std::string&& msg) {
//            if (msg.size() != numBytesRead) {
//                throw MobitoppMessageParseError("Parsed message has size " + std::to_string(msg.size()) + " but " +
//                                                std::to_string(numBytesRead) + " bytes were read.", msg);
//            }
    if (!endsWith(msg, "\n")) {
        throw MobitoppMessageParseError("Message does not end in '\\n'", msg);
    }

    msg = msg.substr(0, msg.size() - 1);

    try {
        auto msgJson = nlohmann::json::parse(msg);
        return msgJson;
    } catch (const nlohmann::json::parse_error &e) {
        throw MobitoppMessageParseError("Message could not be parsed to JSON.", msg);
    }
}