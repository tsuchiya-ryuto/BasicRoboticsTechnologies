#ifndef NODES_H
#define NODES_H

#include <memory>
#include <cmath>

template <typename T>
struct Node
{
    T x,y;
    float cost_so_far, cost_to_go;
    std::shared_ptr<Node> parent;

    Node() : x(0), y(0), cost_so_far(0.0), cost_to_go(0), parent(nullptr) {}
    Node(T xx, T yy) 
    : x(xx), y(yy), cost_so_far(0.0), cost_to_go(0.0), parent(nullptr) {}

    double cost() const { return cost_so_far + cost_to_go; }

    bool operator==(const Node<T>& n) const
    {
        double esp = 1e-6;
        if (fabs(x - n.x) < esp && fabs(y - n.y) < esp)
            return true;
        else
            return false;
    }
};

template <typename T>
struct Action
{
    T dx, dy;

    Action() : dx(0), dy(0) {}
    Action(T dx, T dy) : dx(dx), dy(dy) {}
};

#endif // NODES_H