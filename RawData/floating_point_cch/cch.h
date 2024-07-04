#include <algorithm>
#include <array>
#include <cassert>
#include <vector>
#include "util.h"
#include "bits.h"


namespace floating_point_cch {

    constexpr uint32_t UP = 0;
    constexpr uint32_t DOWN = 1;
    constexpr uint32_t ANY = 0;

    struct CCH {
        std::vector<node_id> order, rank, in_tail, in_head, parent;
        std::vector<edge_id> first_out, in, mapping;
        std::vector<id_t> first_in;
        std::vector<node_id> head, tail;

        CCH() {}

        CCH(const std::vector<node_id> &order_,
            const std::vector<node_id> &tail_,
            const std::vector<node_id> &head_,
            bool reorder = true) : order(order_) {//, in_tail(tail_), in_head(head_) {

            assert(in_tail.size() == in_head.size());

            edge_id edge_count;
            std::vector<std::vector<id_t>> tmp(node_count());
            for (auto &v: tmp) v.reserve(10);
            for (int it: {0, 1}) {
                rank = inverse(order);
                in_tail = tail_;
                in_head = head_;
                apply_perm(rank, in_tail);
                apply_perm(rank, in_head);

                //direct all edges upwards
                for (auto &v: tmp) v.clear();
                for (edge_id i = 0; i < input_arc_count(); i++) {
                    //assert(in_tail[i] != in_head[i]);
                    if (in_tail[i] != in_head[i]) {
                        auto [from, to] = std::minmax(in_tail[i], in_head[i]);
                        tmp[from].push_back(to);
                    }
                }

                //cordal completion
                parent.assign(node_count(), invalid_id);
                edge_count = 0;
                for (node_id i = 0; i < node_count(); i++) {
                    if (!tmp[i].empty()) {
                        //remove duplicates (and sort)
                        std::sort(std::begin(tmp[i]), std::end(tmp[i]), std::greater<>{});
                        auto it = std::unique(std::begin(tmp[i]), std::end(tmp[i]));
                        tmp[i].erase(it, std::end(tmp[i]));

                        edge_count += tmp[i].size();

                        // find parent (last since sorted reverse)
                        parent[i] = tmp[i].back();
                        assert(parent[i] > i);

                        //copy neighbourhood to parent (except the parent himself)
                        tmp[parent[i]].insert(std::end(tmp[parent[i]]), std::begin(tmp[i]),
                                              std::prev(std::end(tmp[i])));
                    }
                }
                if (!reorder) break;
                if (it == 0) {
                    std::vector<std::vector<node_id>> in(node_count());
                    std::vector<node_id> stack;
                    for (node_id i = 0; i < node_count(); i++) {
                        if (parent[i] != invalid_id) {
                            in[parent[i]].push_back(i);
                        } else {
                            stack.push_back(i);
                        }
                    }
                    order.clear();
                    while (!stack.empty()) {
                        order.push_back(stack.back());
                        stack.pop_back();
                        for (node_id x: in[order.back()]) stack.push_back(x);
                    }
                    std::reverse(order.begin(), order.end());
                    for (node_id &x: order) x = order_[x];
                }
            }

            //copy cordal graph into up/down
            head.reserve(edge_count);
            tail.reserve(edge_count);
            first_out.assign(node_count() + 1, 0);
            for (node_id i = 0; i < node_count(); i++) {
                for (node_id j: tmp[i]) {
                    head.emplace_back(j);
                    tail.emplace_back(i);
                }
                first_out[i + 1] = head.size();
            }

            //calculate reverse
            for (auto &v: tmp) v.clear();
            for (edge_id i = 0; i < cch_arc_count(); i++) {
                tmp[head[i]].push_back(i);
            }
            in.reserve(edge_count);
            first_in.assign(node_count() + 1, 0);
            for (node_id i = 0; i < node_count(); i++) {
                for (edge_id ix: tmp[i]) {
                    in.emplace_back(ix);
                }
                first_in[i + 1] = in.size();
            }

            //find mapping
            for (auto &v: tmp) v.clear();//still has capacity >= 10
            for (edge_id i = 0; i < input_arc_count(); i++) {
                if (in_tail[i] != in_head[i]) {
                    node_id from = std::min(in_tail[i], in_head[i]);
                    tmp[from].push_back(i);
                }
            }
            //average sice of tmp[x] < 4 => faster than dual sweep
            mapping.assign(input_arc_count(), invalid_id);
            for (edge_id i = 0; i < cch_arc_count(); i++) {
                for (edge_id j: tmp[tail[i]]) {
                    if (std::max(in_head[j], in_tail[j]) == head[i]) mapping[j] = i;
                }
            }
        }

        node_id node_count() const {
            return order.size();
        }

        edge_id input_arc_count() const {
            return in_head.size();
        }

        edge_id cch_arc_count() const {
            return head.size();
        }

    };

    template<typename T>
    using tuple = std::array<T, 2>;
    using tri = tuple<edge_id>;
    constexpr tri deleted = {invalid_id, 0};

    struct CCH_metric {
        CCH *cch;

        tuple<std::vector<edge_id>> first_out;
        tuple<std::vector<node_id>> head, tail;
        tuple<std::vector<weight_t>> weight;
        tuple<std::vector<flow_t>> flow;
        tuple<std::vector<tri>> tris;
        std::vector<tuple<edge_id>> new_id;


        CCH_metric() {}

        CCH_metric(CCH &cch_) : cch(&cch_), first_out{}, head{}, weight{}, flow{}, tris{}, new_id{} {}

        void customize(const std::vector<weight_t> &weights, bool perfect = true) {
            node_id *cch_in_tail = cch->in_tail.data();
            node_id *cch_in_head = cch->in_head.data();
            edge_id *cch_first_out = cch->first_out.data();
            edge_id *cch_in = cch->in.data();
            edge_id *cch_mapping = cch->mapping.data();
            id_t *cch_first_in = cch->first_in.data();
            node_id *cch_head = cch->head.data();
            node_id *cch_tail = cch->tail.data();

            assert(weights.size() == cch->input_arc_count());
            std::vector<edge_id> tmp_id(cch->node_count(), invalid_id);
            std::vector<tuple<weight_t>> tmp(cch->cch_arc_count(), {inf_weight, inf_weight});
            std::vector<tuple<tri>> tmp_tri(cch->cch_arc_count(),
                                            {tri{invalid_id, invalid_id}, tri{invalid_id, invalid_id}});

            for (edge_id i = 0; i < cch->input_arc_count(); i++) {
                edge_id j = cch_mapping[i];
                if (j != invalid_id) {
                    uint32_t ud = cch_in_tail[i] < cch_in_head[i] ? UP : DOWN;
                    tmp[j][ud] = std::min(tmp[j][ud], weights[i]);

                    assert(ud != UP || cch_tail[j] == cch_in_tail[i]);
                    assert(ud != UP || cch_head[j] == cch_in_head[i]);
                    assert(ud != DOWN || cch_head[j] == cch_in_tail[i]);
                    assert(ud != DOWN || cch_tail[j] == cch_in_head[i]);
                }
            }
            for (node_id i = 0; i < cch->node_count(); i++) {
                for (edge_id iy = cch_first_out[i]; iy < cch_first_out[i + 1]; iy++) tmp_id[cch_head[iy]] = iy;

                for (id_t j = cch_first_in[i]; j < cch_first_in[i + 1]; j++) {
                    edge_id xi = cch_in[j];
                    node_id x = cch_tail[xi];
                    assert(cch_head[xi] == i);

                    for (edge_id xy = cch_first_out[x]; xy < cch_first_out[x + 1] && cch_head[xy] > i; xy++) {
                        node_id y = cch_head[xy];
                        edge_id iy = tmp_id[y];

                        if (iy != invalid_id) {
                            //triangle i->x->y&i->iy (x lowest, i mid, y highest)
#pragma GCC unroll 2
                            for (uint32_t ud: {UP, DOWN}) {
                                weight_t lower = tmp[xi][ud ^ 1] + tmp[xy][ud];
                                if (lower < tmp[iy][ud]) {
                                    tmp[iy][ud] = lower;
                                    tmp_tri[iy][ud] = {xi, xy};
                                    assert(xi < iy);
                                    assert(xy < iy);
                                }
                            }
                        }
                    }
                }

                for (edge_id iy = cch_first_out[i]; iy < cch_first_out[i + 1]; iy++) tmp_id[cch_head[iy]] = invalid_id;
            }
            if (perfect) {
                for (node_id i = cch->node_count() - 1; i != invalid_id; i--) {
                    for (edge_id iy = cch_first_out[i]; iy < cch_first_out[i + 1]; iy++) tmp_id[cch_head[iy]] = iy;

                    for (edge_id ix = cch_first_out[i]; ix < cch_first_out[i + 1]; ix++) {
                        node_id x = cch_head[ix];
                        for (edge_id xy = cch_first_out[x]; xy < cch_first_out[x + 1]; xy++) {
                            node_id y = cch_head[xy];
                            edge_id iy = tmp_id[y];

                            if (iy != invalid_id) {
                                //triangle i->ix->xy&i->iy (i lowest, x mid, y highest)
#pragma GCC unroll 2
                                for (uint32_t ud: {UP, DOWN}) {
                                    weight_t upper = tmp[iy][ud] + tmp[xy][ud ^ 1] + EPS;
                                    if (upper < tmp[ix][ud]) {
                                        tmp[ix][ud] = upper;
                                        tmp_tri[ix][ud] = deleted;
                                    }
                                    weight_t mid = tmp[ix][ud] + tmp[xy][ud] + EPS;
                                    if (mid < tmp[iy][ud]) {
                                        tmp[iy][ud] = mid;
                                        tmp_tri[iy][ud] = deleted;
                                    }
                                }
                            }
                        }

                    }

                    for (edge_id iy = cch_first_out[i]; iy < cch_first_out[i + 1]; iy++)
                        tmp_id[cch_head[iy]] = invalid_id;
                }
            }
            //copy data
#pragma GCC unroll 2
            for (uint32_t ud: {UP, DOWN}) {
                first_out[ud].assign(cch->node_count() + 1, 0);
                head[ud].clear();
                head[ud].reserve(cch->cch_arc_count());
                tail[ud].clear();
                tail[ud].reserve(cch->cch_arc_count());
                weight[ud].clear();
                weight[ud].reserve(cch->cch_arc_count());
                flow[ud].clear();
                flow[ud].reserve(cch->cch_arc_count());
                tris[ud].clear();
                tris[ud].reserve(cch->cch_arc_count());
                new_id.assign(cch->cch_arc_count(), {invalid_id, invalid_id});
            }

            for (edge_id i = 0; i < cch->cch_arc_count(); i++) {
#pragma GCC unroll 2
                for (uint32_t ud: {UP, DOWN}) {
                    if (tmp[i][ud] != inf_weight && tmp_tri[i][ud] != deleted) {
                        first_out[ud][cch_tail[i] + 1]++;
                        new_id[i][ud] = head[ud].size();
                        head[ud].push_back(cch_head[i]);
                        tail[ud].push_back(cch_tail[i]);
                        weight[ud].push_back(tmp[i][ud]);
                        flow[ud].push_back(0);
                        auto [a, b] = tmp_tri[i][ud];

                        if (a != invalid_id) {
                            assert(b != invalid_id);

                            assert(a < i);
                            a = new_id[a][ud ^ 1];
                            assert(a < tris[ud ^ 1].size());

                            assert(b < i);
                            b = new_id[b][ud];
                            assert(b < tris[ud].size());

                            assert(head[ud].back() == head[ud][b]);
                            assert(tail[ud].back() == head[ud ^ 1][a]);
                        }
                        tris[ud].push_back(tri{a, b});
                    }
                }
            }
#pragma GCC unroll 2
            for (uint32_t ud: {UP, DOWN}) {
                for (node_id i = 0; i < cch->node_count(); i++) {
                    first_out[ud][i + 1] += first_out[ud][i];
                }
            }
        }

        std::vector<flow_t> get_flow() {
            // propagate flow
            for (node_id i = cch->node_count() - 1; i != invalid_id; i--) {
#pragma GCC unroll 2
                for (uint32_t ud: {UP, DOWN}) {
                    for (edge_id j = first_out[ud][i]; j < first_out[ud][i + 1]; j++) {
                        auto [a, b] = tris[ud][j];
                        if (a != invalid_id) {
                            assert(b != invalid_id);

                            //triangle
                            assert(tail[ud][j] == head[ud ^ 1][a]);
                            assert(head[ud][j] == head[ud][b]);
                            assert(tail[ud ^ 1][a] == tail[ud][b]);

                            flow[ud ^ 1][a] += flow[ud][j];
                            flow[ud][b] += flow[ud][j];
                            flow[ud][j] = 0;
                        }
                    }
                }
            }

            // map flow weights
            node_id *cch_in_tail = cch->in_tail.data();
            node_id *cch_in_head = cch->in_head.data();
            edge_id *cch_mapping = cch->mapping.data();
            std::vector<flow_t> in_flow(cch->input_arc_count());

            for (edge_id i = 0; i < cch->input_arc_count(); i++) {
                edge_id j = cch_mapping[i]; // input edge-id to cch-edge-id
                if (j == invalid_id) continue;
                uint32_t ud = cch_in_tail[i] < cch_in_head[i] ? UP : DOWN;
                j = new_id[j][ud]; // input cch-edge-id to perfect customized id
                if (j == invalid_id) continue;
                assert(ud != UP || tail[ud][j] == cch_in_tail[i]);
                assert(ud != UP || head[ud][j] == cch_in_head[i]);
                assert(ud != DOWN || head[ud][j] == cch_in_tail[i]);
                assert(ud != DOWN || tail[ud][j] == cch_in_head[i]);
                in_flow[i] += flow[ud][j];
            }
            return in_flow;
        }

    };

    using path_t = std::vector<node_id>;
    using edge_path_t = std::vector<edge_id>;

    struct CCH_query {
        CCH_metric *cch_metric;
        CCH *cch;
        std::vector<tuple<weight_t>> dist;
        std::vector<tuple<edge_id>> prev;
        node_id lca, meet, s, t;

        CCH_query() {}

        CCH_query(CCH_metric &cch_metric_) :
                cch_metric(&cch_metric_),
                cch(cch_metric->cch),
                dist(cch->node_count(), {inf_weight, inf_weight}),
                prev(cch->node_count()) {}

    private:
        template<typename P, typename C>
        void find_paths(node_id s_, node_id t_, P &&prune, C &&callback) {
            s = cch->rank[s_];
            t = cch->rank[t_];

            node_id *parent = cch->parent.data();
            std::vector<edge_id> *first_out = cch_metric->first_out.data();
            std::vector<node_id> *head = cch_metric->head.data();
            std::vector<weight_t> *weight = cch_metric->weight.data();

            tuple<node_id> cur = {s, t};
            dist[s][UP] = dist[t][DOWN] = 0;
            prev[s][UP] = prev[t][DOWN] = invalid_id;

            //advance smaller vertex
            while (cur[UP] != cur[DOWN]) {
                uint32_t ud = cur[UP] <= cur[DOWN] ? UP : DOWN;
                node_id c = cur[ud];
                if (dist[c][ud] != inf_weight) {
                    for (edge_id i = first_out[ud][c]; i < first_out[ud][c + 1]; i++) {
                        node_id x = head[ud][i];
                        weight_t d = dist[c][ud] + weight[ud][i];
                        if (d < dist[x][ud]) {
                            dist[x][ud] = d;
                            prev[x][ud] = i;
                        }
                    }
                    dist[c][ud] = inf_weight;
                }
                cur[ud] = parent[c];
            }

            node_id c = cur[ANY];
            lca = c;
            //advance both vertices
            for (; c != invalid_id; c = parent[c]) {
                callback(c, dist[c][UP] + dist[c][DOWN]);
#pragma GCC unroll 2
                for (uint32_t ud: {UP, DOWN}) {
                    if (!prune(dist[c][ud])) {
                        for (edge_id i = first_out[ud][c]; i < first_out[ud][c + 1]; i++) {
                            node_id x = head[ud][i];
                            weight_t d = dist[c][ud] + weight[ud][i];
                            if (d < dist[x][ud]) {
                                dist[x][ud] = d;
                                prev[x][ud] = i;
                            }
                        }
                    }
                    dist[c][ud] = inf_weight;
                }
            }
        }

    public:

        weight_t query(node_id s_, node_id t_) {
            weight_t res = inf_weight;
            node_id meet_ = invalid_id;
            find_paths(s_, t_,
                       [&res](weight_t d) {//prune
                           return d >= res;
                       },
                       [&res, &meet_](node_id c, weight_t d) {//new path callback
                           if (d < res) {
                               meet_ = c;
                               res = d;
                           }
                       });
            meet = meet_;
            return res;
        }

    private:
        template<uint32_t UD>
        void unpack(path_t &path, edge_id e) const {
            auto [a, b] = cch_metric->tris[UD][e];
            if (a == invalid_id) {
                assert(b == invalid_id);
                if constexpr (UD == UP) path.push_back(cch_metric->head[UD][e]);
                else path.push_back(cch_metric->tail[UD][e]);
            } else {
                assert(cch_metric->head[UD][e] == cch_metric->head[UD][b]);
                assert(cch_metric->tail[UD][e] == cch_metric->head[UD ^ 1][a]);
                assert(cch_metric->tail[UD][b] == cch_metric->tail[UD ^ 1][a]);
                assert(cch_metric->weight[UD][e] == cch_metric->weight[UD][b] + cch_metric->weight[UD ^ 1][a]);
                if constexpr (UD == UP) {
                    unpack<UD ^ 1>(path, a);
                    unpack<UD>(path, b);
                } else {
                    unpack<UD>(path, b);
                    unpack<UD ^ 1>(path, a);
                }
            }
        }

        path_t raw_path(node_id mid) const {
            if (mid == invalid_id) return {};
            std::vector<node_id> *tail = cch_metric->tail.data();

            path_t with_shortcuts, res;
            with_shortcuts.reserve(100);
            with_shortcuts.push_back(mid);
            while (prev[with_shortcuts.back()][UP] != invalid_id) {
                edge_id in = prev[with_shortcuts.back()][UP];
                with_shortcuts.push_back(tail[UP][in]);
            }
            res.push_back(with_shortcuts.back());
            with_shortcuts.pop_back();
            std::reverse(std::begin(with_shortcuts), std::end(with_shortcuts));

            for (node_id c: with_shortcuts) {
                unpack<UP>(res, prev[c][UP]);
            }
            while (prev[res.back()][DOWN] != invalid_id) {
                unpack<DOWN>(res, prev[res.back()][DOWN]);
            }
            return res;
        }

        void map_path(path_t &path) const {
            node_id *order = cch->order.data();
            for (node_id &x: path) x = order[x];
        }

    public:

        path_t path() const {
            path_t res = raw_path(meet);
            map_path(res);
            return res;
        }

        void add_flow(flow_t amount) {
            if (meet == invalid_id) return;
            std::vector<node_id> *tail = cch_metric->tail.data();
            std::vector<flow_t> *flow = cch_metric->flow.data();

#pragma GCC unroll 2
            for (uint32_t ud: {UP, DOWN}) {
                node_id c = meet;
                while (prev[c][ud] != invalid_id) {
                    edge_id in = prev[c][ud];
                    c = tail[ud][in];
                    flow[ud][in] += amount;
                }
            }
        }
    };

} // namespace floating_point_cch