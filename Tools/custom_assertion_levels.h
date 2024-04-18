#pragma once

namespace kassert::assert {
#define KASSERT_ASSERTION_LEVEL_LIGHT 20
    constexpr int light = KASSERT_ASSERTION_LEVEL_LIGHT;

#define KASSERT_ASSERTION_LEVEL_HEAVY 40
    constexpr int heavy = KASSERT_ASSERTION_LEVEL_HEAVY;
} // namespace kassert::assert