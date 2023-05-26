#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>
#include <functional>

using namespace std;
template<typename Vertex , typename Distance = double>
class Graph {

public:
    struct Edge {
        Vertex from, to;
        Distance distance;
    };

private:
    unordered_map<Vertex, vector<Edge>> _data;

public:
    //проверка-добавление-удаление вершин

    Graph(){}

    ~Graph() {
        if (!_data.empty()) { _data.clear(); }
    }

    bool has_vertex(const Vertex& v) const {
        for (const auto& vertex : _data) {
            if (v == vertex.first) { return true; }
        }
        return false;
    }

    void add_vertex(const Vertex& v) {
         if (has_vertex(v)) { throw std::invalid_argument("The graph already has a given vertex!"); }
         _data.insert({ v, {} });
    }

    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v)) { return false; }
        _data.erase(v);
        return true;
    }

    vector<Vertex> vertices() const {
        vector<Vertex> result;
        for (auto& i : _data) {
            result.push_back(i.first);
        }
        return result;
    }

    unique_ptr<vector<Vertex>> vertices_ptr() const {
        vector<Vertex> res;
        for (auto& i : _data) {
            res.push_back(i.first);
        }
        return make_unique<vector<Vertex>>(res);
    }

    //проверка-добавление-удаление ребер
    void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
        if (!has_vertex(from) or ! has_vertex(to)) { throw std::invalid_argument("The graph has no one of two vertex!"); }
        _data.find(from)->second.push_back({ from, to, d });
    }

    bool remove_edge(const Vertex& from, const Vertex& to) {
        if (!has_vertex(from) or !has_vertex(to)) { return false; }
        auto& edges = _data.find(from)->second;
        auto it = edges.begin();
        bool is_any_erases = false;

        while (it != edges.end()) {
            if (it->from == from and it->to == to) {
                it = edges.erase(it);
                is_any_erases = true;
            }
            else { ++it; }
        }
        return is_any_erases;
    }
    bool remove_edge(const Edge& e) {//c учетом расстояния
        if (!has_vertex(e.from) or !has_vertex(e.to)) { return false; }
        auto& edges = _data.find(e.from)->second;
        auto it = edges.begin();
        bool is_any_erases = false;

        while (it != edges.end()) {
            if (it->from == e.from and it->to == e.to and it->distance == e.distance) {
                it = edges.erase(it);
                is_any_erases = true;
            }
            else { ++it; }
        }
        return is_any_erases;
    }

    bool has_edge(const Vertex& from, const Vertex& to) const {
        if (!has_vertex(from) or !has_vertex(to)) { return false; }
        for (auto& edge : edges(from)) {
            if (edge.from == from and edge.to = to) { return true; }
        }
        return false;
    }

    bool has_edge(const Edge& e) const {//c учетом расстояния в Edge
        if (!has_vertex(e.from) or !has_vertex(e.to)) { return false; }
        for (auto& edge : edges(e.from)) {
            if (edge.from == e.from and edge.to == e.to and edge.distance == e.distance) { return true; }
        }
        return false;
    }
    //получение всех ребер, выходящих из вершины
    vector<Edge> edges(const Vertex& v) const {
        if (!has_vertex(v)) { throw std::invalid_argument("The graph has no given vertex!"); }
        vector<Edge> result;
        auto& edges = _data.find(v)->second;
        auto it = edges.begin();

        while (it != edges.end()) {
            result.push_back(*it);
            ++it;
        }
        return result;
    }

    unique_ptr<vector<Edge>> edges_ptr(const Vertex& v) const {
        if (!has_vertex(v)) throw std::invalid_argument("The graph has no given vertex!");
        vector<Edge> result;
        auto& edges = _data.find(v)->second;
        auto it = edges.begin();
        while (it != edges.end()) {
            result.push_back(*it);
            ++it;
        }
        return make_unique<vector<Edge>>(result);
    }

    size_t order() const {//порядок
        return _data.size();
    }
    size_t degree(const Vertex& v) const {//степень
        if (!has_vertex(v)) { throw std::invalid_argument("The graph has no given vertex!"); }
        return _data.find(v)->second.size();
    }

    bool is_it_has_negative_cycle(const unordered_map<Vertex,Distance>& distances) const {
        for (const auto& v : vertices()) {
            const auto& edges_v = edges(v);
            for (auto it = edges_v.cbegin(); it != edges_v.cend(); ++it) {
                const auto& e = *it;
                if (distances.at(e.from) + e.distance < distances.at(e.to)) {
                    return true;
                }
            }
        }
        return false;
    }

    //поиск кратчайшего пути
    vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        unordered_map<Vertex, Distance> distances;
        unordered_map<Vertex, Vertex> predecessors;

        for (const auto& v : _data) {
            distances[v.first] = numeric_limits<Distance>::infinity();
        }
        distances[from] = 0;

        for (size_t i = 0; i < _data.size() - 1; ++i) {
            for (const auto& vertices : _data) {
                for (const auto& edges : vertices.second) {
                    if (distances[edges.from] + edges.distance < distances[edges.to]) {
                        distances[edges.to] = distances[edges.from] + edges.distance;
                        predecessors[edges.to] = edges.from;
                    }
                }
            }
        }

        if (is_it_has_negative_cycle(distances)) { throw std::logic_error("Graph contains a negative-weight cycle"); }

        vector<Vertex> path;
        Vertex current = to;
        while (current != from) {
            path.push_back(current);
            current = predecessors[current];
        }
        path.push_back(from);
        reverse(path.begin(), path.end());

        Distance current_distance = 0;

        vector<Edge> result;
        current = path[0];
        size_t i = 0;
        while (current != to) {
            Vertex next = path.at(i);
            auto& edges = _data.find(current)->second;
            for (auto& edge : edges) {
                if (edge.to == _data.find(next)->first and edge.distance + current_distance == distances[next]) {
                    result.push_back(edge);
                    current_distance += edge.distance;
                }
            }
            current = next;
            ++i;
        }
        return result;
    }

    void dfs(unordered_map<Vertex, bool>& visited, const Vertex& current_vertex, vector<Vertex>& result) {
        visited[current_vertex] = true;
        result.push_back(current_vertex);
        for (const auto& edge : _data[current_vertex]) {
            const auto& next_vertex = edge.to;
            if (!visited[next_vertex]) {
                dfs(visited, next_vertex, result);
            }
        }
    }

    vector<Vertex> walk(const Vertex& start_vertex) {
        unordered_map<Vertex, bool> visited;
        vector<Vertex> result;
        dfs(visited, start_vertex, result);
        return result;
    }

    void walk_(unordered_map<Vertex, bool>& visited, const Vertex& current_vertex, function<void(const Vertex&)> action) {
        visited[current_vertex] = true;
        action(current_vertex);
        for (const auto& edge : _data[current_vertex]) {
            const auto& next_vertex = edge.to;
            if (!visited[next_vertex]) {
                walk_(visited, next_vertex, action);
            }
        }
    }

    void walk(const Vertex& start_vertex, function<void(const Vertex&)> action) {
        unordered_map<Vertex, bool> visited;
        walk_(visited, start_vertex, action);
    }

    Distance average_distance(const unordered_map<Vertex, Distance> distances) const {
        Distance sum = 0;
        size_t size = distances.size();
        for (const auto& distance : distances) {
            sum += distance.second;
        }
        return sum / size;
    }

    Vertex task() const {
        bool negative_cycle_flag = true;
        Vertex store_vertex{};
        Distance min_distance = numeric_limits<Distance>::infinity();

        for (const auto& vertex : _data) {
            unordered_map<Vertex, Distance> distances;

            for (const auto& v : _data) {
                distances[v.first] = numeric_limits<Distance>::infinity();
            }
            distances[vertex.first] = 0;

            for (size_t i = 0; i < _data.size() - 1; ++i) {
                for (const auto& vertices : _data) {
                    for (const auto& edge : vertices.second) {
                        if (distances[edge.from] + edge.distance < distances[edge.to]) {
                            distances[edge.to] = distances[edge.from] + edge.distance;
                        }
                    }
                }
            }

            if (average_distance(distances) < min_distance) {
                store_vertex = vertex.first;
            }

            if (negative_cycle_flag) {
                if (is_it_has_negative_cycle(distances)) { throw std::logic_error("Graph contains a negative-weight cycle"); }
                negative_cycle_flag = false;
            }
        }

        return store_vertex;
    }
      
};

template<typename Vertex>
void print(const Vertex& vert) {
    cout << vert << " ";
}

int main() {
    Graph<size_t, double> d;
    d.add_vertex(4);
    d.add_vertex(3);
    d.add_vertex(1);
    d.add_vertex(6);

    d.add_edge(4, 3, 5);

    d.add_edge(4, 6, 8);
    Graph<size_t, double> g;
    g.add_vertex(1);
    g.add_vertex(2);
    g.add_vertex(3);
    g.add_vertex(4);
    g.add_edge(1, 2, 1.0);
    g.add_edge(1, 3, 1.0);
    g.add_edge(2, 4, 1.0);
    g.add_edge(3, 4, 1.0);
    g.has_vertex(1);
    g.edges(1);
    vector<size_t> result = g.walk(1);
    for (auto i : result) {
        cout << i << ",";
    }
    cout << endl;

    Graph<size_t, double> ex;
    ex.add_vertex(1);
    ex.add_vertex(2);
    ex.add_vertex(3);
    ex.add_vertex(4);
    ex.add_vertex(5);

    ex.add_edge(1, 2, 3);
    ex.add_edge(1, 3, 4);
    ex.add_edge(2, 3, -2);
    ex.add_edge(2, 4, 2);
    ex.add_edge(3, 5, 6);
    ex.add_edge(4, 5, 1);

    for (auto& i : ex.shortest_path(1, 5)) {
        cout << i.from << "->" << i.to << " [distance : " << i.distance << "]" << endl;
    }

    cout << ex.task() << endl;;

    ex.walk(1, print<size_t>);

    auto vertices = d.vertices_ptr();
    for (auto& i : *vertices.get()) {
        cout << i << " ";
    }
    cout << endl;

    auto edges = ex.edges_ptr(1);
    for (auto& i : *edges.get()) {
        cout << i.from << "->" << i.to << " ";
    }
    cout << endl;
    return 0;
}