#ifndef NODE_H
#define NODE_H

#include <cmath>  // For std::hypot

class Node {
private:
    Node* parent;  // Pointer to the parent node

    float x, y, theta; // location + angle if needed
    float fScore;

public:
    // Constructor
    Node(Node* parent = nullptr, float x = 0, float y = 0, float theta = 0, float fScore = 0);

    // Getter methods
    Node* getParent() const;
    float getX() const;
    float getY() const;
    float getTheta() const;
    double getFScore() const;

    // Setter methods
    void setParent(Node* parent);
    void setX(float x);
    void setY(float y);
    void setTheta(float theta);
    void setFScore(double fScore);

    // Comparison operator for priority queues (min-heap comparison)
    bool operator<(const Node& other) const;

    // Equality operator to compare positions
    bool operator==(const Node& other) const;

    // Calculate the Euclidean distance between this node and another node
    float distanceTo(const Node& other) const;
};

#endif // NODE_H
