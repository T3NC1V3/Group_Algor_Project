#include <iostream>
#include <vector>
#include <climits>

using namespace std;

#define NUM_NODES 23
#define INF INT_MAX

// Function to find the node with the minimum distance value
int minDistance(const vector<int>& dist, const vector<bool>& visited) {
    int minDist = INF, minIndex;
    for (int i = 0; i < NUM_NODES; ++i) {
        if (!visited[i] && dist[i] <= minDist) {
            minDist = dist[i];
            minIndex = i;
        }
    }
    return minIndex;
}

// Function to print the shortest paths from start to all nodes
void printShortcuts(const vector<int>& dist, const vector<int>& parent, char startNode) {
    cout << "Shortest paths from node " << startNode << " to all nodes:\n";
    for (int i = 0; i < NUM_NODES; ++i) {
        cout << "To node " << char('A' + i) << ": Distance = " << dist[i] << ", Path = ";
        int current = i;
        while (current != -1) {
            cout << char('A' + current) << " ";
            current = parent[current];
        }
        cout << endl;
    }
}

// Dijkstra's algorithm to find the shortest paths
void dijkstra(const vector<vector<int>>& graph, int startNode) {
    vector<int> distances(NUM_NODES, INF); // Initialize distances to nodes as infinite
    vector<bool> visited(NUM_NODES, false); // Initialize nodes as unvisited
    vector<int> parent(NUM_NODES, -1); // Array to store shortest path tree

    distances[startNode] = 0; // Distance from start node to itself is 0

    for (int count = 0; count < NUM_NODES - 1; ++count) {
        int u = minDistance(distances, visited); // Get the node with the least distance
        visited[u] = true; // Mark node as visited

        // Update distance value of the adjacent nodes of the current node
        for (int v = 0; v < NUM_NODES; ++v) {
            if (!visited[v] && graph[u][v] && distances[u] != INF && distances[u] + graph[u][v] < distances[v]) {
                distances[v] = distances[u] + graph[u][v]; // Update distance
                parent[v] = u; // Update parent
            }
        }
    }

    // Print the shortest paths from start to all nodes
    printShortcuts(distances, parent, char('A' + startNode));
}

int main() {
    // Adjacency matrix
    vector<vector<int>> graph = {
     // {A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W} (for reference, staring at a graph to copy values was pain) 
        {0, 6, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // A
        {6, 0, 5, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // B
        {0, 5, 0, 7, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // C
        {0, 0, 7, 0, 7, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // D
        {0, 0, 0, 7, 0, 0, 0, 0, 6, 0, 0, 0, 0, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // E
        {5, 0, 0, 0, 0, 0, 8, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // F
        {0, 6, 0, 0, 0, 8, 0, 9, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // G
        {0, 0, 5, 0, 0, 0, 9, 0, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // H
        {0, 0, 0, 8, 6, 0, 0, 12, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // I
        {0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 5, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0}, // J
        {0, 0, 0, 0, 0, 0, 8, 0, 0, 5, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // K
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 7, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0}, // L
        {0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 7, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // M
        {0, 0, 0, 0, 15, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0}, // N
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 13, 0, 0, 9, 0, 0, 0, 0}, // O
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 13, 0, 8, 0, 0, 0, 11, 0, 0}, // P
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 9, 0, 0, 0, 0, 0}, // Q
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 9, 0, 0, 0, 0, 0, 10}, // R
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 9, 0, 0, 0}, // S
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 8, 0, 0}, // T
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 0, 0, 0, 8, 0, 8, 0}, // U
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 5}, // V
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 5, 0}, // W
    };

    // Start node 
    int startNode = 0; // Starting from node A

    // Dijkstra's
    dijkstra(graph, startNode);

    return 0;
}