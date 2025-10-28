// graph.h
#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <string>

struct Node {
    int64_t id;
    double lat;
    double lon;

    Node() = default;
    Node(int64_t id_, double lat_, double lon_) : id(id_), lat(lat_), lon(lon_) {}
};

class Graph {
public:
    std::unordered_map<int64_t, Node> nodes;
    std::unordered_map<int64_t, std::vector<std::pair<int64_t, double>>> adj;

    void load_from_osm(const std::string& filename);
    std::vector<int64_t> a_star(int64_t start_id, int64_t goal_id);
    int64_t find_nearest_node(double lat, double lon);

private:
    double haversine(double lat1, double lon1, double lat2, double lon2);
};

#endif // GRAPH_H
