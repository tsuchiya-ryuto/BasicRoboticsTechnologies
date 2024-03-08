#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <nodes.h>

#include <iostream>
#include <unordered_map>
#include <queue>
#include <cmath>

using namespace std;

namespace PathPlanning
{
class AStar
{
public:
    AStar();
	~AStar();

	void set_display_configure(bool save, FILE *gnuplot);
    void set_map(const vector<vector<int>>& grid_map);
    bool planning(const Node& start, const Node& goal);
    float heuristic_cost(const Node& goal, const Node& node);
    void calculate_path(const Node& goal);

    bool is_valid_index(const int& ix, const int& iy) const;
    bool is_collision(const Node& node) const;
	void display_node(const Node& n);
    
    int get_id(const Node& node);
    vector<Node> get_path();
    vector<Action> get_action();


private:
    vector<vector<int>> map;
    int height, width;
    vector<Node> path;
	vector<Node> all_path;
    
	bool enable_display;
	FILE *gp;
};
}
#endif // ASTAR_H
