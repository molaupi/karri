#include <vector>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <limits>

namespace floating_point_cch {
    using weight_t = float;
    using flow_t = uint32_t; //integral
    using capacity_t = uint32_t; //integral
    using id_t = uint32_t;
    using node_id = id_t;
    using edge_id = id_t;

    constexpr weight_t EPS = 0.2;

    constexpr weight_t inf_weight = std::numeric_limits<weight_t>::infinity();
    constexpr id_t invalid_id = 0xFFFF'FFFF;

    static_assert(inf_weight + inf_weight >= inf_weight, "inf_weight is too large");
    static_assert(sizeof(unsigned) == 4, "code base assumes that unsigned is 32 bit wide");

    std::vector<id_t> inverse(const std::vector<id_t> &in) {
        std::vector<id_t> res(in.size(), invalid_id);
        for (id_t i = 0; i < in.size(); i++) {
            assert(in[i] < in.size());
            assert(res[in[i]] == invalid_id);
            res[in[i]] = i;
        }
        return res;
    }

    void apply_perm(const std::vector<id_t> &perm, std::vector<id_t> &in) {
        for (id_t &x: in) x = perm[x];
    }

    template<typename T>
    void loadVector(const std::string &fileName, std::vector<T> &vec) {
        std::ifstream is(fileName, std::ios::binary);
        assert(is);
        assert(is.is_open());
        is.seekg(0, std::ios::end);
        unsigned long long file_size = is.tellg();
        assert(file_size % sizeof(T) == 0);
        is.seekg(0, std::ios::beg);
        vec.resize(file_size / sizeof(T));
        is.read(reinterpret_cast<char *>(&vec[0]), file_size);
    }

    template<class T>
    std::vector<T> loadVector(const std::string &fileName) {
        std::vector<T> vec;
        loadVector(fileName, vec);
        return vec;
    }

    template<class T>
    inline void dumpVector(const std::string &fileName, const std::vector<T> &vec) {
        std::ofstream os(fileName, std::ios::binary);
        assert(os);
        assert(os.is_open());
        os.write(reinterpret_cast<const char *>(&vec[0]), vec.size() * sizeof(T));
    }

    std::vector<node_id> tail_from_first_out(const std::vector<edge_id> first_out) {
        std::vector<node_id> tail(first_out.back());
        for (node_id i = 0; i + 1 < first_out.size(); i++) {
            for (edge_id j = first_out[i]; j < first_out[i + 1]; j++) {
                tail[j] = i;
            }
        }
        return tail;
    }

}