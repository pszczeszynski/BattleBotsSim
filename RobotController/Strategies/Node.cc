#include "Node.h"
#include <functional>
#include <cmath>

// Constructor
Node::Node(Node* parent = nullptr, float x = 0, float y = 0, float theta = 0, float fScore = 0)
    : parent(parent), x(x), y(y), theta(theta), fScore(fScore) {}

// Getter methods
Node* Node::getParent() const {
    return parent;
}

float Node::getX() const {
    return x;
}

float Node::getY() const {
    return y;
}

float Node::getTheta() const {
    return theta;
}

double Node::getFScore() const {
    return fScore;
}

// Setter methods
void Node::setParent(Node* parent) {
    this->parent = parent;
}

void Node::setX(float x) {
    this->x = x;
}

void Node::setY(float y) {
    this->y = y;
}

void Node::setTheta(float theta) {
    this->theta = theta;
}

void Node::setFScore(double fScore) {
    this->fScore = fScore;
}

// Min-heap comparison based on fScore
bool Node::operator<(const Node& other) const {
    return fScore > other.fScore;  // Higher fScore means lower priority
}

// Compare if two nodes have the same (x, y) position
bool Node::operator==(const Node& other) const {
    return x == other.x && y == other.y;
}

// Calculate the Euclidean distance to another node
float Node::distanceTo(const Node& other) const {
    return std::hypot(other.x - x, other.y - y);
}


// Custom hash function for Node
namespace std {
    template <>
    struct hash<Node> {
        std::size_t operator()(const Node& node) const {
            std::size_t h1 = std::hash<float>{}(node.getX());
            std::size_t h2 = std::hash<float>{}(node.getY());
            return h1 ^ (h2 << 1); // Combine the hashes
        }
    };
}
