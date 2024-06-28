#ifndef __PROGTEST__
#include <cassert>
#include <cctype>
#include <cmath>
#include <cinttypes>
#include <limits>

#include <memory>
#include <string>
#include <random>
#include <algorithm>

#include <iostream>
#include <iomanip>

#include <vector>
#include <deque>
#include <list>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <stack>

// rooms are numbered consecutively starting with zero
using Room = unsigned;
using Corridor = std::pair<Room, Room>;

inline constexpr Room NO_ROOM = std::numeric_limits<Room>::max();

struct Map {
    Room s;
    std::vector<Room> beds;
    std::vector<unsigned> food;
    std::vector<Corridor> corridors;
};

struct Result {
    Room final_bed;
    unsigned food_eaten;
};

bool operator == (Result a, Result b) {
    return a.final_bed == b.final_bed && (
            a.food_eaten == b.food_eaten || a.final_bed == NO_ROOM);
}

std::ostream& operator << (std::ostream& out, Result r) {
    out << "(room: ";
    if (r.final_bed == NO_ROOM) out << "NO_ROOM";
    else out << r.final_bed;
    return out << ", food: " << r.food_eaten << ")";
}

#endif
inline constexpr size_t NO_COMP = std::numeric_limits<size_t>::max();


template<typename T_room = Room>
struct DirectedGraph {
    std::vector<std::vector<T_room>> neighbors;

    DirectedGraph(size_t n, const std::vector<std::pair<T_room, T_room>> &paths)
        : neighbors(n) {
        for(auto [from, to]: paths)
            neighbors[from].emplace_back(to);
    }

    [[nodiscard]] DirectedGraph T() const {
        DirectedGraph reversed(size(), {});

        for(T_room from = 0; from < size(); from++)
            for(auto to: neighbors[from])
                reversed.neighbors[to].emplace_back(from);
        return reversed;
    }

    [[nodiscard]] const std::vector<T_room> &operator[](T_room room) const {
        return neighbors[room];
    }

    [[nodiscard]] std::vector<T_room> &operator[](T_room room) {
        return neighbors[room];
    }

    [[nodiscard]] size_t size() const {
        return neighbors.size();
    }
};

template<typename T_room = Room, typename T_weight = unsigned>
struct VWDirectedGraph: public DirectedGraph<T_room> {
    std::vector<T_weight> weights;

    VWDirectedGraph(DirectedGraph<T_room> G, std::vector<T_weight> weights)
            : DirectedGraph<T_room>(std::move(G)), weights(std::move(weights))
    {}

    VWDirectedGraph(
            size_t n,
            const std::vector<std::pair<T_room, T_room>> &paths,
            std::vector<T_weight> weights
    ) : VWDirectedGraph(DirectedGraph(n, paths), std::move(weights))
    {}

//    using DirectedGraph<T_room>::T;
};

template<typename T_component = size_t,
         typename T_weight = unsigned,
         typename T_room = Room>
struct CondensedVWGraph: public VWDirectedGraph<T_component, T_weight> {
    std::vector<std::pair<T_room, size_t>> comp_room;

    CondensedVWGraph()
        : VWDirectedGraph<T_component, T_weight>(0, {}, {})
    {}

    void addComponent(std::vector<T_component> c_ngb,
                      T_weight c_weight,
                      T_room repr,
                      size_t repr_idx) {
        DirectedGraph<T_component>::neighbors.emplace_back(std::move(c_ngb));
        VWDirectedGraph<T_component, T_weight>::weights.emplace_back(c_weight);
        comp_room.emplace_back(repr, repr_idx);
    }
};


//void dfs_close(std::vector<bool> &visited,
//               const DirectedGraph<Room> &GT,
//               std::vector<Room> &close_order,
//               Room room) {
//    visited[room] = true;
//
//    for(auto ngb: GT[room])
//        if(!visited[ngb])
//            dfs_close(visited, GT, close_order, ngb);
//
//    close_order.emplace_back(room);
//}

void dfs_iter(std::vector<bool> &visited,
              DirectedGraph<Room> &GT,
              std::vector<Room> &close_order,
              Room start) {
    std::vector<Room> stack;
    stack.push_back(start);

    nest: while(!stack.empty()) {
        auto curr = stack.back();
        visited[curr] = true;

        while(!GT[curr].empty()) {
            auto ngb = GT[curr].back();
            GT[curr].pop_back();

            if(!visited[ngb]) {
                stack.push_back(ngb);
                goto nest;
            }
        }

        close_order.push_back(curr);
        stack.pop_back();
    }
}

// condensation, start_component
std::pair<CondensedVWGraph<>, size_t>
condense(const Map& map) {
    const size_t N = map.food.size();
    VWDirectedGraph G{N, map.corridors, map.food};

    std::vector<Room> close_order;
    close_order.reserve(N);
    {
        auto GT = G.T();
        std::vector<bool> visited(N);

        for(Room start = 0; start < N; start++)
            if(!visited[start])
                dfs_iter(visited, GT, close_order, start);
    }

    CondensedVWGraph CG;
    std::vector<Room> room_repr_idx(N, NO_ROOM);
    for(size_t idx = 0; idx < map.beds.size(); idx++)
        room_repr_idx[map.beds[idx]] = idx;

    std::vector<size_t> components(N, NO_COMP);

    size_t current_component = 0;
    for(auto it = close_order.rbegin(); it != close_order.rend(); it++) {
        auto start = *it;
        if(components[start] != NO_COMP)
            continue;

        std::vector<size_t> component_ngb;
        unsigned c_weight = 0;
        Room room_repr = NO_ROOM;

        std::vector<Room> dfs_stack;
        components[start] = current_component;
        dfs_stack.push_back(start);

        while(!dfs_stack.empty()) {
            auto curr = dfs_stack.back();
            dfs_stack.pop_back();

            c_weight += G.weights[curr];
            if(room_repr == NO_ROOM) {
                room_repr = (room_repr_idx[curr] == NO_ROOM) ? NO_ROOM : curr;
            }
            else if(room_repr_idx[curr] < room_repr_idx[room_repr])
                room_repr = curr;

            for(auto ngb: G[curr]) {
                if(components[ngb] == NO_COMP) {
                    components[ngb] = current_component;
                    dfs_stack.push_back(ngb);
                }
                else if(components[ngb] < current_component)
                    component_ngb.emplace_back(components[ngb]);
            }
        }

        CG.addComponent(component_ngb,
                        c_weight,
                        room_repr,
                        room_repr != NO_ROOM ? room_repr_idx[room_repr] : NO_COMP);

        current_component++;
    }

    return {CG, components[map.s]};
}


template<typename T_component, typename T_weight>
std::pair<T_component, T_weight>
relax_search(const CondensedVWGraph<T_component, T_weight> &CG, T_component start) {
    const T_weight NO_D = std::numeric_limits<T_weight>::max();
    std::vector<T_weight> D(CG.size(), NO_D);
    std::vector<bool> open(CG.size());
    std::queue<T_component> q;

    auto enqueue = [&q, &open, &D](T_component v, T_weight d) -> void {
        D[v] = d;
        if(!open[v]) {
            open[v] = true;
            q.push(v);
        }
    };

    auto dequeue = [&q, &open]() -> T_component {
        auto curr = q.front();
        q.pop();
        open[curr] = false;
        return curr;
    };

    T_component best_comp = NO_COMP;
    T_weight best_weight = 0;

    enqueue(start, CG.weights[start]);
    while(!q.empty()) {
        auto curr = dequeue();
        auto cd = D[curr];

        if(CG.comp_room[curr].first != NO_ROOM && (
            cd > best_weight ||
            (cd == best_weight && (
                best_comp == NO_COMP ||
                CG.comp_room[best_comp].second >= CG.comp_room[curr].second))
                )) {
            best_comp = curr;
            best_weight = cd;
        }

        for(auto ngb: CG[curr])
            if(D[ngb] == NO_D || D[ngb] < D[curr] + CG.weights[ngb])
                enqueue(ngb, D[curr] + CG.weights[ngb]);
    }

    return {best_comp, best_weight};
}


Result find_lost_cat(const Map& map) {
    auto [CG, c_start] = condense(map);
    auto [comp, food] = relax_search<size_t, unsigned>(CG, c_start);

    if(comp == NO_COMP)
        return {NO_ROOM, 0};

    return {CG.comp_room[comp].first, food};
}


#ifndef __PROGTEST__


const std::vector<std::pair<Result, Map>> test_data = {
  // Simple
  { { 1, 4 }, Map{ 0, { 1 },
    { 0, 1, 1, 2, 1 },
    {
      { 0, 2 }, { 0, 3 },
      { 2, 4 }, { 3, 4 },
      { 0, 1 }, { 4, 1 }
    }
  }},
  { { 7, 8 }, Map{ 3, { 7 },
    {
       1,  1,  0,  0,  1,  2,  0,  0,  0,  1,  2,  2,
       2,
    }, {
      { 0,  2}, { 1,  8}, { 2,  4}, { 2,  7}, { 2, 11}, { 3,  6},
      { 3,  9}, { 3, 10}, { 4,  7}, { 5,  8}, { 6,  1}, { 6,  5},
      { 6,  8}, { 8,  0}, { 8,  2}, { 8, 12}, { 9,  6}, {10,  6},
      {11,  7}, {12,  2},
    }
  }},
  { { NO_ROOM, 0 }, Map{ 4, { 5 },
    {
       2,  1,  0,  0,  2,  0,  2,  2,  1,  0,  1,  0,
       1,
    }, {
      { 0,  2}, { 1,  5}, { 3,  0}, { 3,  2}, { 3, 10}, { 4,  3},
      { 5,  6}, { 5,  8}, { 5, 11}, { 6, 11}, { 7,  5}, { 8, 11},
      { 9,  1}, { 9,  5}, { 9,  7}, {10,  2}, {11,  3}, {11,  4},
      {11, 12}, {12,  3},
    }
  }},

  // Acyclic
  { { NO_ROOM, 0 }, Map{ 2, { 4 },
    {
       6,  1,  3,  0,  3,  6,  4,  0,
    }, {
      { 0,  1}, { 0,  6}, { 1,  6}, { 3,  0}, { 3,  2}, { 3,  6},
      { 4,  1}, { 4,  5}, { 5,  0}, { 5,  1}, { 5,  6}, { 6,  2},
      { 7,  2}, { 7,  3}, { 7,  4},
    }
  }},
  { { NO_ROOM, 0 }, Map{ 3, { 7 },
    {
       0,  0,  0,  2,  0,  6,  6,  0,
    }, {
      { 1,  0}, { 1,  3}, { 1,  5}, { 2,  1}, { 3,  5}, { 4,  2},
      { 5,  6}, { 7,  3}, { 7,  4}, { 7,  5}, { 7,  6},
    }
  }},
  { { 6, 6 }, Map{ 0, { 6 },
    {
       2,  2,  2,  1,  0,  0,  4,  6,
    }, {
      { 0,  5}, { 0,  6}, { 2,  1}, { 3,  2}, { 4,  1}, { 4,  2},
      { 5,  7}, { 6,  1}, { 6,  2}, { 6,  4}, { 6,  5}, { 6,  7},
      { 7,  1},
    }
  }},

  // General
  { { 8, 23 }, Map{ 1, { 8 },
    {
       3,  6,  4,  0,  6,  4,  6,  3,  0,  3,
    }, {
      { 0,  8}, { 1,  5}, { 2,  6}, { 2,  9}, { 3,  0}, { 3,  2},
      { 3,  8}, { 4,  2}, { 4,  8}, { 5,  3}, { 5,  8}, { 6,  0},
      { 7,  2}, { 7,  3}, { 8,  1}, { 8,  9},
    }
  }},
  { { NO_ROOM, 0 }, Map{ 6, { 7 },
    {
       5,  0,  0,  1,  5,  0,  0,  0,  2,  5,
    }, {
      { 0,  4}, { 0,  9}, { 1,  4}, { 2,  3}, { 3,  5}, { 3,  6},
      { 4,  0}, { 5,  8}, { 6,  1}, { 6,  4}, { 7,  0}, { 7,  1},
      { 7,  8}, { 9,  4},
    }
  }},
  { { 6, 14 }, Map{ 5, { 6 },
    {
       5,  0,  1,  3,  2,  3,  6,  5,  0,  0,
    }, {
      { 0,  4}, { 0,  6}, { 1,  0}, { 1,  5}, { 2,  4}, { 3,  7},
      { 4,  2}, { 5,  0}, { 5,  1}, { 5,  4}, { 6,  4}, { 7,  9},
      { 8,  4}, { 8,  6}, { 9,  8},
    }
  }},
  { { 2, 15 }, Map{ 1, { 2 },
    {
       1,  0,  6,  0,  0,  0,  5,  0,  5,  4,
    }, {
      { 1,  3}, { 1,  7}, { 3,  2}, { 3,  7}, { 3,  9}, { 4,  5},
      { 4,  7}, { 5,  3}, { 5,  8}, { 7,  3}, { 7,  8}, { 8,  1},
      { 8,  6}, { 9,  5}, { 9,  8},
    }
  }},
  { { NO_ROOM, 0 }, Map{ 9, { 3 },
    {
       1,  4,  2,  3,  0,  1,  0,  4,  1,  0,
    }, {
      { 0,  1}, { 1,  0}, { 1,  5}, { 1,  9}, { 2,  0}, { 2,  5},
      { 2,  7}, { 3,  9}, { 4,  9}, { 6,  2}, { 6,  3}, { 6,  5},
      { 7,  0}, { 7,  2}, { 7,  3}, { 7,  8}, { 8,  2}, { 9,  4},
    }
  }},
};

int main() {
  for (const auto& [ exp_res,  M ] : test_data) {
    Result stud_res = find_lost_cat(M);
    if (stud_res != exp_res) {
      std::cout << "Fail: " << exp_res << " != " << stud_res << std::endl;
    }
  }

  return 0;
}

#endif


