#include <astar.h>

using namespace std;

namespace PathPlanning
{

AStar::AStar() : enable_display(false), gp(nullptr) {}
AStar::~AStar() {}


void AStar::set_display_configure(bool save, FILE *gnuplot)
{
    enable_display = save;
    gp = gnuplot;
}

void AStar::set_map(const vector<vector<int>>& grid_map) 
{ 
    this->map = grid_map;
    height = map.size();
    width = map[0].size();
}

bool AStar::planning(const Node& start, const Node& goal)
{
    if(is_collision(start))
    {
        cout << "Start is in collision" << endl;
        return false;
    }

    if(is_collision(goal))
    {
        cout << "Goal is in collision" << endl;
        return false;
    }

    auto compare_cost = [](Node& lhs, Node& rhs)
    {
        return (
            (lhs.cost() > rhs.cost()) ||
            (lhs.cost() == rhs.cost() && lhs.cost_to_go > rhs.cost_to_go)
        );
    };

    priority_queue<Node, vector<Node>, decltype(compare_cost)> open_set(compare_cost);
    unordered_map<int, Node> visited_nodes;

            if(enable_display)
                display_node(start);
    
            // start planning
    open_set.push(start);
    auto actions = get_action();

    do
    {
        shared_ptr<Node> current(new Node);
        *current = open_set.top();
        open_set.pop();
                    
                    //if(enable_display)
                    //	display_node(*current);

        int current_id = get_id(*current);
        visited_nodes.insert_or_assign(current_id, *current);
        
        if(*current == goal)
        {
                            if(enable_display)
                                fprintf(gp, "e\n");
            calculate_path(*current);
            return true;
        }

        for(auto& action : actions)
        {
            int x,y;
            x = current->x + action.dx;
            y = current->y + action.dy;
            Node next(x,y);
            next.parent = current;
            int next_id = get_id(next);

            if(!is_valid_index(x, y))
                continue;
            
            if(is_collision(next))
                continue;
            
            float action_cost;
            action_cost = sqrt(pow(action.dx, 2) + pow(action.dy, 2));
            next.cost_so_far = current->cost_so_far + action_cost;
            next.cost_to_go = heuristic_cost(goal, next);

            if(visited_nodes.find(next_id) != visited_nodes.end())
            {
                auto previous_it = visited_nodes.find(next_id);
                Node previous = previous_it->second;
                if(next.cost_so_far < previous.cost_so_far)
                {
                    visited_nodes.erase(next_id);
                    visited_nodes.insert_or_assign(next_id, next);
                }
                else
                    continue;
            }
                            if(enable_display)
                                display_node(next);
                                
            open_set.push(next);
        }
    } while (!open_set.empty());
    
            if(enable_display)
                fprintf(gp, "e\n");

    return false;
}

float AStar::heuristic_cost(const Node& goal, const Node& node)
{
    return abs(goal.x - node.x) + abs(goal.y - node.y);
}

void AStar::calculate_path(const Node& goal)
{
    Node node = goal;
    path.push_back(goal);
    do{
        node = *node.parent;
        path.insert(path.begin(), node);
    }while(node.parent != nullptr);
}

vector<Node> AStar::get_path() { return path; }

vector<Action> AStar::get_action()
{
    vector<Action> actions {
            {-1, 1}, {0, 1}, {1, 1},
            {-1, 0},          {1, 0},
            {-1, -1}, {0, -1}, {1, -1}
        };
    return actions;
}

void AStar::display_node(const Node& n)
{
	fprintf(gp, "%d %d 0.5\n", n.x, n.y);
}

bool AStar::is_valid_index(const int& ix, const int& iy) const
{
    return (ix >= 0 && ix < width) && (iy >= 0 && iy < height);
}

bool AStar::is_collision(const Node& node) const
{ 
    return map.at(node.y).at(node.x) == 1;
}

int AStar::get_id(const Node& node) { return node.x + node.y * width; }

} // namespace PathPlanning
