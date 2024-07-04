#include <vector>
#include <cstdint>

namespace floating_point_cch {

    class bits {
        std::vector<uint64_t> data;
        std::vector<std::size_t> has_data;

    public:
        bits() {}

        bits(std::size_t n) : data(n) {
            has_data.reserve((n >> 6) + 1);
        }

        bool operator[](std::size_t i) const {
            std::size_t a = i >> 6;
            std::size_t b = i & 0b111111;
            return (data[a] & (1ull << b)) != 0;
        }

        void set(std::size_t i) {
            std::size_t a = i >> 6;
            std::size_t b = i & 0b111111;
            if (data[a] == 0) has_data.push_back(a);
            data[a] |= 1ull << b;
        }

        void reset() {
            std::sort(has_data.begin(), has_data.end());
            for (std::size_t i: has_data) data[i] = 0;
            has_data.clear();
        }
    };

} // namespace floating_point_cch