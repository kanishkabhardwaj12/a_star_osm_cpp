#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/geom/haversine.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/tags/filter.hpp>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include "graph.h"

using namespace std;

double Graph::haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371e3;
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double delta_phi = (lat2 - lat1) * M_PI / 180.0;
    double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(delta_phi/2.0) * sin(delta_phi/2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda/2.0) * sin(delta_lambda/2.0);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

int64_t Graph::find_nearest_node(double lat, double lon) {
    int64_t nearest_id = -1;
    double min_dist = numeric_limits<double>::max();

    for (const auto& [id, node] : nodes) {
        double dist = haversine(lat, lon, node.lat, node.lon);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_id = id;
        }
    }
    return nearest_id;
}

vector<int64_t> Graph::a_star(int64_t start_id, int64_t goal_id) {
    unordered_map<int64_t, double> g_score;
    unordered_map<int64_t, double> f_score;
    unordered_map<int64_t, int64_t> came_from;
    auto cmp = [&](int64_t left, int64_t right) {
        return f_score[left] > f_score[right];
    };
    priority_queue<int64_t, vector<int64_t>, decltype(cmp)> open_set(cmp);

    g_score[start_id] = 0;
    f_score[start_id] = haversine(nodes[start_id].lat, nodes[start_id].lon,
                                  nodes[goal_id].lat, nodes[goal_id].lon);
    open_set.push(start_id);

    while (!open_set.empty()) {
        int64_t current = open_set.top();
        open_set.pop();
        if (current == goal_id) {
            vector<int64_t> path;
            while (came_from.count(current)) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start_id);
            reverse(path.begin(), path.end());
        
            double total_distance = 0.0;
            for (size_t i = 1; i < path.size(); ++i) {
                Node& prev = nodes[path[i - 1]];
                Node& curr = nodes[path[i]];
                total_distance += haversine(prev.lat, prev.lon, curr.lat, curr.lon);
            }
        
            cout << "Total distance (meters): " << total_distance << endl;
        
            return path;
        }

        for (auto [neighbor, cost] : adj[current]) {
            double tentative_g = g_score[current] + cost;
            if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                f_score[neighbor] = tentative_g + haversine(
                    nodes[neighbor].lat, nodes[neighbor].lon,
                    nodes[goal_id].lat, nodes[goal_id].lon);
                open_set.push(neighbor);
            }
        }
    }
    return {};
}

void Graph::load_from_osm(const string& filename) {
    using index_type = osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location>;
    index_type index;
    osmium::handler::NodeLocationsForWays<index_type> location_handler(index);

    struct MyHandler : public osmium::handler::Handler {
        Graph& graph;
        MyHandler(Graph& g) : graph(g) {}

        void node(const osmium::Node& node) {
            if (!node.location().valid()) return;
            graph.nodes[node.id()] = Node(node.id(), node.location().lat(), node.location().lon());
        }

        void way(const osmium::Way& way) {
            const char* highway = way.tags()["highway"];
            if (!highway) return;

            const auto& nodes = way.nodes();
            for (size_t i = 1; i < nodes.size(); ++i) {
                int64_t from = nodes[i - 1].ref();
                int64_t to = nodes[i].ref();
                if (!graph.nodes.count(from) || !graph.nodes.count(to)) continue;

                double dist = graph.haversine(
                    graph.nodes[from].lat, graph.nodes[from].lon,
                    graph.nodes[to].lat, graph.nodes[to].lon);

                graph.adj[from].emplace_back(to, dist);
                graph.adj[to].emplace_back(from, dist);
            }
        }
    };

    osmium::io::Reader reader(filename);
    MyHandler handler(*this);
    osmium::apply(reader, location_handler, handler);
    reader.close();
}