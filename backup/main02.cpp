//Dijkstra normal
#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <queue>
#include <iostream>
#include <algorithm>

using namespace std;
using namespace sf;

const int WIDTH = 800;
const int HEIGHT = 600;
const int NODE_SPACING = 20;

struct Node {
    Vector2f pos;
    bool obstacle = false;
    int index;
};

struct Edge {
    int to;
    float cost;
    Edge(int t, float c) : to(t), cost(c) {}
};

struct State {
    int node;
    float cost;
    State(int n, float c) : node(n), cost(c) {}
    bool operator<(const State& other) const {
        return cost > other.cost;
    }
};

int getIndex(int x, int y, int cols) {
    return y * cols + x;
}

int main() {
    RenderWindow window(VideoMode(WIDTH, HEIGHT), "Dijkstra Visual");

    int cols = WIDTH / NODE_SPACING;
    int rows = HEIGHT / NODE_SPACING;
    int totalNodes = cols * rows;

    vector<Node> nodes(totalNodes);
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int idx = getIndex(x, y, cols);
            nodes[idx].pos = Vector2f(x * NODE_SPACING + NODE_SPACING / 2.f, y * NODE_SPACING + NODE_SPACING / 2.f);
            nodes[idx].index = idx;
        }
    }

    vector<vector<Edge>> graph(totalNodes);
    auto valid = [&](int x, int y) { return x >= 0 && y >= 0 && x < cols && y < rows; };

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int fromIdx = getIndex(x, y, cols);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx;
                    int ny = y + dy;
                    if (valid(nx, ny)) {
                        int toIdx = getIndex(nx, ny, cols);
                        float dist = sqrt(dx * dx + dy * dy) * NODE_SPACING;
                        graph[fromIdx].push_back(Edge(toIdx, dist));
                    }
                }
            }
        }
    }

    CircleShape agent(8);
    agent.setFillColor(Color::Blue);
    int startNode = getIndex(5, 5, cols);
    Vector2f agentPos = nodes[startNode].pos;

    vector<int> path;
    size_t pathIndex = 0;
    vector<int> visitedNodes;

    auto dijkstra = [&](int start, int goal) -> vector<int> {
        visitedNodes.clear();
        vector<float> dist(totalNodes, 1e9f);
        vector<int> cameFrom(totalNodes, -1);
        priority_queue<State> pq;

        dist[start] = 0;
        pq.push(State(start, 0));

        while (!pq.empty()) {
            State current = pq.top(); pq.pop();
            if (current.node == goal) break;

            if (current.cost > dist[current.node]) continue;

            visitedNodes.push_back(current.node);

            for (auto& edge : graph[current.node]) {
                int nextNode = edge.to;
                if (nodes[nextNode].obstacle) continue;
                float newCost = dist[current.node] + edge.cost;
                if (newCost < dist[nextNode]) {
                    dist[nextNode] = newCost;
                    cameFrom[nextNode] = current.node;
                    pq.push(State(nextNode, newCost));
                }
            }
        }

        vector<int> reversePath;
        if (cameFrom[goal] == -1) return {};
        for (int cur = goal; cur != -1; cur = cameFrom[cur]) {
            reversePath.push_back(cur);
        }
        reverse(reversePath.begin(), reversePath.end());
        return reversePath;
    };

    while (window.isOpen()) {
        Event event;
        while (window.pollEvent(event)) {
            if (event.type == Event::Closed)
                window.close();

            if (event.type == Event::MouseButtonPressed) {
                int mx = event.mouseButton.x;
                int my = event.mouseButton.y;
                int gx = mx / NODE_SPACING;
                int gy = my / NODE_SPACING;
                if (valid(gx, gy)) {
                    int clickedNode = getIndex(gx, gy, cols);
                    if (event.mouseButton.button == Mouse::Left) {
                        nodes[clickedNode].obstacle = !nodes[clickedNode].obstacle;
                    } else if (event.mouseButton.button == Mouse::Right) {
                        int currentAgentNode = getIndex((int)(agentPos.x / NODE_SPACING), (int)(agentPos.y / NODE_SPACING), cols);
                        path = dijkstra(currentAgentNode, clickedNode);
                        pathIndex = 0;
                    }
                }
            }
        }

        if (pathIndex < path.size()) {
            Vector2f targetPos = nodes[path[pathIndex]].pos;
            Vector2f dir = targetPos - agentPos;
            float len = sqrt(dir.x * dir.x + dir.y * dir.y);
            if (len > 1.0f) {
                dir /= len;
                agentPos += dir * 2.5f;
            } else {
                agentPos = targetPos;
                pathIndex++;
            }
        }

        window.clear();

        for (auto& node : nodes) {
            RectangleShape rect(Vector2f(NODE_SPACING - 1, NODE_SPACING - 1));
            rect.setOrigin(NODE_SPACING / 2.f, NODE_SPACING / 2.f);
            rect.setPosition(node.pos);

            if (node.obstacle)
                rect.setFillColor(Color::Red);
            else
                rect.setFillColor(Color(70, 70, 70));

            window.draw(rect);
        }

        for (int idx : visitedNodes) {
            RectangleShape rect(Vector2f(NODE_SPACING - 1, NODE_SPACING - 1));
            rect.setOrigin(NODE_SPACING / 2.f, NODE_SPACING / 2.f);
            rect.setPosition(nodes[idx].pos);
            rect.setFillColor(Color(255, 140, 0, 100));
            window.draw(rect);
        }

        for (int i = 0; i + 1 < path.size(); i++) {
            Vertex line[] = {
                Vertex(nodes[path[i]].pos, Color::Green),
                Vertex(nodes[path[i + 1]].pos, Color::Green)
            };
            window.draw(line, 2, Lines);
        }

        agent.setPosition(agentPos - Vector2f(agent.getRadius(), agent.getRadius()));
        window.draw(agent);

        window.display();
    }

    return 0;
}
