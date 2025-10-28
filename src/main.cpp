#include <iostream>
#include <fstream>
#include "graph.h" 

using namespace std;

int main(int argc, char** argv) {
    std::cout << "Program started." << std::endl;

    if (argc != 2) {
        cout << "missing osm file name." << endl;
        return 1;
    }

    string osm_file = argv[1];
    Graph graph;
    graph.load_from_osm(osm_file);
    cout << "OSM data loaded." << endl;


    //jaypee 
    double start_lat = 28.630627;
    double start_lon = 77.373626;
    //fortis
    double end_lat = 28.619028;
    double end_lon = 77.373173;

    int64_t start_node = graph.find_nearest_node(start_lat, start_lon);
    int64_t end_node = graph.find_nearest_node(end_lat, end_lon);
    // if (start_node == -1 || end_node == -1) { 
    //     cerr << "[Error] Could not find valid nearest start or end node!" << endl; return 1; 
    // }
    //cout << "End Node ID (raw): " << end_node << endl; cout << "Node exists in graph.nodes? " << (graph.nodes.count(end_node) > 0) << endl;
    //if (graph.nodes.count(end_node)) { cout << "Lat: " << graph.nodes[end_node].lat << " Lon: " << graph.nodes[end_node].lon << endl; }
    cout << "Start Node ID: " << start_node << endl;
    cout << "End Node ID: " << end_node << endl;

    if (start_node == -1 || end_node == -1) {
        cout << "Could not find nearest node for the given coordinates." << endl;
        return 1;
    }
    cout << "Start node neighbors: " << graph.adj[start_node].size() << endl;
    cout << "End node neighbors: " << graph.adj[end_node].size() << endl;

    vector<int64_t> path = graph.a_star(start_node, end_node);

    cout << "Shortest path node IDs:\n";
    for (auto node_id : path) {
        cout << node_id << " ";
    }
    cout << endl;

    std::ofstream out("path.json");
    if (!out) {
        cerr << "Error: Could not open file for writing path JSON.\n";
        return 1;
    }
    out << "[\n";
    for (size_t i = 0; i < path.size(); ++i) {
        const auto& node = graph.nodes[path[i]];
        out << "  { \"lat\": " << node.lat << ", \"lon\": " << node.lon << " }";
        if (i + 1 != path.size()) out << ",";
        out << "\n";
    }
    out << "]\n";
    out.close();
    cout << "Path written to path.json\n";

    return 0;
}
