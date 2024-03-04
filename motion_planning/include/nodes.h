#ifndef NODES_H
#define NODES_H

#include <memory>

struct Node
{
    int x,y;
    float cost_so_far, cost_to_go;
    std::shared_ptr<Node> parent;

    Node() : x(0), y(0), cost_so_far(0.0), cost_to_go(0), parent(nullptr) {}
    Node(int xx, int yy) 
    : x(xx), y(yy), cost_so_far(0.0), cost_to_go(0.0), parent(nullptr) {}

    double cost() const { return cost_so_far + cost_to_go; }

    bool operator==(const Node& n) const
    {
        return x == n.x && y == n.y;
    }
};

struct Action
{
    int dx, dy;

    Action() : dx(0), dy(0) {}
    Action(int dx, int dy) : dx(dx), dy(dy) {}
};

#endif // NODES_H