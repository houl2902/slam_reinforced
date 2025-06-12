#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iostream>

class Map {
private:
    int width;
    int height;
    std::vector<std::vector<bool>> grid; // true means obstacle

public:
    Map(int w, int h) : width(w), height(h), grid(h, std::vector<bool>(w, false)) {}

    void setObstacle(int x, int y, bool isObstacle) {
        if (x < 0  x >= width  y < 0 || y >= height) {
            throw std::out_of_range("Map coordinates out of range");
        }
        grid[y][x] = isObstacle;
    }

    bool isObstacle(int x, int y) const {
        if (x < 0  x >= width  y < 0 || y >= height) {
            return true; // Consider out of bounds as obstacle
        }
        return grid[y][x];
    }

    int getWidth() const { return width; }
    int getHeight() const { return height; }

    void printMap() const {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                std::cout << (grid[y][x] ? 'X' : '.');
            }
            std::cout << std::endl;
        }
    }
};

class AStar {
    private:
        struct Node {
            int x, y;
            double g, h, f;
            Node* parent;
    
            Node(int x, int y, double g, double h, Node* parent = nullptr)
                : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}
    
            bool operator>(const Node& other) const {
                return f > other.f;
            }
        };
    
        Map map;
        double heuristic(int x1, int y1, int x2, int y2) {
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        }
    
    public:
        AStar(int width, int height) : map(width, height) {}
    
        void setObstacle(int x, int y, bool isObstacle) {
            map.setObstacle(x, y, isObstacle);
        }
    
        std::vector<std::pair<int, int>> findPath(int startX, int startY, int goalX, int goalY) {
            if (map.isObstacle(startX, startY) || map.isObstacle(goalX, goalY)) {
                return {}; // No path if start or goal is obstacle
            }
    
            std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
            std::vector<std::vector<bool>> closedSet(map.getHeight(), std::vector<bool>(map.getWidth(), false));
            std::vector<std::vector<Node*>> nodes(map.getHeight(), std::vector<Node*>(map.getWidth(), nullptr));
    
            Node* startNode = new Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY));
            openSet.push(*startNode);
            nodes[startY][startX] = startNode;
    
            const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
            const int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};
    
            while (!openSet.empty()) {
                Node current = openSet.top();
                openSet.pop();
    
                if (current.x == goalX && current.y == goalY) {
                    // Reconstruct path
                    std::vector<std::pair<int, int>> path;
                    Node* node = nodes[current.y][current.x];
                    while (node != nullptr) {
                        path.emplace_back(node->x, node->y);
                        node = node->parent;
                    }
                    std::reverse(path.begin(), path.end());
    
                    // Clean up
                    for (int y = 0; y < map.getHeight(); ++y) {
                        for (int x = 0; x < map.getWidth(); ++x) {
                            delete nodes[y][x];
                        }
                    }
                    return path;
                }
    
                closedSet[current.y][current.x] = true;
    
                for (int i = 0; i < 8; ++i) {
                    int nx = current.x + dx[i];
                    int ny = current.y + dy[i];
    
                    if (nx < 0  nx >= map.getWidth()  ny < 0  ny >= map.getHeight()  
                        map.isObstacle(nx, ny) || closedSet[ny][nx]) {
                        continue;
                    }
    
                    double newG = current.g + ((i < 4) ? 1.0 : 1.414); // sqrt(2) for diagonal
                    double newH = heuristic(nx, ny, goalX, goalY);
                    double newF = newG + newH;
    
                    if (nodes[ny][nx] == nullptr || newF < nodes[ny][nx]->f) {
                        if (nodes[ny][nx] != nullptr) {
                            // Need to update priority in openSet, but priority_queue doesn't support it
                            // So we just add a new node (the old one will be ignored when we find it in closedSet)
                        }
                        Node* newNode = new Node(nx, ny, newG, newH, nodes[current.y][current.x]);
                        openSet.push(*newNode);
                        nodes[ny][nx] = newNode;
                    }
                }
            }
    
            // Clean up
            for (int y = 0; y < map.getHeight(); ++y) {
                for (int x = 0; x < map.getWidth(); ++x) {
                    delete nodes[y][x];
                }
            }
    
            return {}; // No path found
        }
    
        void printMapWithPath(int startX, int startY, int goalX, int goalY) {
            auto path = findPath(startX, startY, goalX, goalY);
            
            for (int y = 0; y < map.getHeight(); ++y) {
                for (int x = 0; x < map.getWidth(); ++x) {
                    if (x == startX && y == startY) {
                        std::cout << 'S';
                    } else if (x == goalX && y == goalY) {
                        std::cout << 'G';
                    } else if (map.isObstacle(x, y)) {
                        std::cout << 'X';
                    } else if (std::find(path.begin(), path.end(), std::make_pair(x, y)) != path.end()) {
                        std::cout << '*';
                    } else {
                        std::cout << '.';
                    }
                }
                std::cout << std::endl;
            }
        }
    };