// Visualización paso a paso de Dijkstra
#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <queue>
#include <iostream>
#include <algorithm>
#include <limits>

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
    RenderWindow window(VideoMode(WIDTH, HEIGHT), "Dijkstra Paso a Paso");

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
    Vector2f agentPos;
    
    vector<int> path;
    size_t pathIndex = 0;
    vector<int> visitedNodes;

    // Variables para Dijkstra paso a paso
    priority_queue<State> pq;
    vector<float> dist;
    vector<int> cameFrom;
    int startNode = -1, goalNode = -1;
    bool runningDijkstra = false;
    Clock clock;

    auto initDijkstra = [&](int start, int goal) {
        visitedNodes.clear();
        dist.assign(totalNodes, numeric_limits<float>::infinity());
        cameFrom.assign(totalNodes, -1);
        while (!pq.empty()) pq.pop();
        dist[start] = 0;
        pq.push(State(start, 0));
        startNode = start;
        goalNode = goal;
        runningDijkstra = true;
    };

    auto stepDijkstra = [&]() {
        if (pq.empty()) {
            runningDijkstra = false;
            return;
        }
        State current = pq.top(); pq.pop();
        if (current.cost > dist[current.node]) return;

        visitedNodes.push_back(current.node);
        if (current.node == goalNode) {
            runningDijkstra = false;
            path.clear();
            for (int cur = goalNode; cur != -1; cur = cameFrom[cur])
                path.push_back(cur);
            reverse(path.begin(), path.end());
            pathIndex = 0;
            return;
        }

        for (auto& edge : graph[current.node]) {
            int next = edge.to;
            if (nodes[next].obstacle) continue;
            float newCost = dist[current.node] + edge.cost;
            if (newCost < dist[next]) {
                dist[next] = newCost;
                cameFrom[next] = current.node;
                pq.push(State(next, newCost));
            }
        }
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
                        int currentAgentNode = getIndex(agentPos.x / NODE_SPACING, agentPos.y / NODE_SPACING, cols);
                        initDijkstra(currentAgentNode, clickedNode);
                    }
                }
            }
        }

        if (runningDijkstra && clock.getElapsedTime().asMilliseconds() >= 1) {
            stepDijkstra();
            clock.restart();
        }

        if (pathIndex < path.size()) {
            Vector2f target = nodes[path[pathIndex]].pos;
            Vector2f dir = target - agentPos;
            float len = sqrt(dir.x * dir.x + dir.y * dir.y);
            if (len > 1.0f) {
                dir /= len;
                agentPos += dir * 2.5f;
            } else {
                agentPos = target;
                pathIndex++;
            }
        }

        window.clear();
        for (auto& node : nodes) {
            RectangleShape rect(Vector2f(NODE_SPACING - 1, NODE_SPACING - 1));
            rect.setOrigin(NODE_SPACING / 2.f, NODE_SPACING / 2.f);
            rect.setPosition(node.pos);
            rect.setFillColor(node.obstacle ? Color::Red : Color(70, 70, 70));
            window.draw(rect);
        }

        for (int idx : visitedNodes) {
            RectangleShape rect(Vector2f(NODE_SPACING - 1, NODE_SPACING - 1));
            rect.setOrigin(NODE_SPACING / 2.f, NODE_SPACING / 2.f);
            rect.setPosition(nodes[idx].pos);
            rect.setFillColor(Color(255, 140, 0, 100));
            window.draw(rect);
        }

        for (size_t i = 0; i + 1 < path.size(); i++) {
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