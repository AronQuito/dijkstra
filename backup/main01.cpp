#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <queue>
#include <iostream>
#include <algorithm>

const int WIDTH = 800;
const int HEIGHT = 600;
const int NODE_SPACING = 20;

struct Node {
    sf::Vector2f pos;
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

int getIndex(int x, int y, int cols) { //crea un index para cada valor en un array.
    return y * cols + x;
}

int main() {
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Dijkstra con visualización");

    int cols = WIDTH / NODE_SPACING;
    int rows = HEIGHT / NODE_SPACING;
    int totalNodes = cols * rows;

    std::vector<Node> nodes(totalNodes);
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int idx = getIndex(x, y, cols);
            nodes[idx].pos = sf::Vector2f(x * NODE_SPACING + NODE_SPACING / 2.f, y * NODE_SPACING + NODE_SPACING / 2.f);
            nodes[idx].index = idx;
        }
    }

    std::vector<std::vector<Edge>> graph(totalNodes);
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
                        float dist = std::sqrt(dx * dx + dy * dy) * NODE_SPACING;
                        graph[fromIdx].push_back(Edge(toIdx, dist));
                    }
                }
            }
        }
    }

    sf::CircleShape agent(8);
    agent.setFillColor(sf::Color::Blue);
    int startNode = getIndex(5, 5, cols);
    sf::Vector2f agentPos = nodes[startNode].pos;

    std::vector<int> path;
    size_t pathIndex = 0;
    std::vector<int> visitedNodes;

    auto dijkstra = [&](int start, int goal) -> std::vector<int> { //DIJKSTRA ALGORITHM
        visitedNodes.clear();
        std::vector<float> dist(totalNodes, 1e9f);
        std::vector<int> cameFrom(totalNodes, -1);
        std::priority_queue<State> pq;

        dist[start] = 0;
        pq.push(State(start, 0));

        while (!pq.empty()) {
            State current = pq.top(); pq.pop();
            if (current.node == goal) break;

            if (current.cost > dist[current.node]) continue;

            visitedNodes.push_back(current.node); // registrar nodo visitado

            for (auto& edge : graph[current.node]) { // explora vecinos
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

        std::vector<int> reversePath;
        if (cameFrom[goal] == -1) return {};
        for (int cur = goal; cur != -1; cur = cameFrom[cur]) {
            reversePath.push_back(cur);
        }
        std::reverse(reversePath.begin(), reversePath.end());
        return reversePath;
    };

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed) {
                int mx = event.mouseButton.x;
                int my = event.mouseButton.y;
                int gx = mx / NODE_SPACING;
                int gy = my / NODE_SPACING;
                if (valid(gx, gy)) {
                    int clickedNode = getIndex(gx, gy, cols);
                    if (event.mouseButton.button == sf::Mouse::Left) {
                        nodes[clickedNode].obstacle = !nodes[clickedNode].obstacle;
                    } else if (event.mouseButton.button == sf::Mouse::Right) {
                        int currentAgentNode = getIndex((int)(agentPos.x / NODE_SPACING), (int)(agentPos.y / NODE_SPACING), cols);
                        path = dijkstra(currentAgentNode, clickedNode);
                        pathIndex = 0;
                    }
                }
            }
        }

        if (pathIndex < path.size()) {
            sf::Vector2f targetPos = nodes[path[pathIndex]].pos;
            sf::Vector2f dir = targetPos - agentPos;
            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
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
            sf::RectangleShape rect(sf::Vector2f(NODE_SPACING - 1, NODE_SPACING - 1));
            rect.setOrigin(NODE_SPACING / 2.f, NODE_SPACING / 2.f);
            rect.setPosition(node.pos);

            if (node.obstacle)
                rect.setFillColor(sf::Color::Red);
            else
                rect.setFillColor(sf::Color(70, 70, 70)); // gris oscuro por defecto

            window.draw(rect);
        }

        // Dibujar nodos visitados (amarillo claro)
        for (int idx : visitedNodes) {
            sf::RectangleShape rect(sf::Vector2f(NODE_SPACING - 1, NODE_SPACING - 1));
            rect.setOrigin(NODE_SPACING / 2.f, NODE_SPACING / 2.f);
            rect.setPosition(nodes[idx].pos);
            rect.setFillColor(sf::Color(255, 255, 0, 100)); // amarillo transparente
            window.draw(rect);
        }

        // Dibujar camino final (verde)
        for (int i = 0; i + 1 < path.size(); i++) {
            sf::Vertex line[] = {
                sf::Vertex(nodes[path[i]].pos, sf::Color::Green),
                sf::Vertex(nodes[path[i + 1]].pos, sf::Color::Green)
            };
            window.draw(line, 2, sf::Lines);
        }

        agent.setPosition(agentPos - sf::Vector2f(agent.getRadius(), agent.getRadius()));
        window.draw(agent);

        window.display();
    }
    return 0;
}
