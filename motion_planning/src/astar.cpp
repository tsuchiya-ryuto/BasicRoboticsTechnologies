#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <astar.h>

using namespace std;

vector<string> split(string& input, char delimiter)
{
    istringstream stream(input);
    string field;
    vector<string> result;
    while (getline(stream, field, delimiter))
        result.push_back(field);
    return result;
}

void load_grid_map(vector<vector<int>>& map, std::string file_name) {
  std::ifstream ifs(file_name);
  if(ifs)
  {
    std::string line;
    while(getline(ifs, line))
    {
        vector<int> datvec;
        vector<string> strvec = split(line, ',');
        for(auto &&s : strvec)
            datvec.push_back(stoi(s));
        map.push_back(datvec);
    }
  }
}

int main(int argc, char** argv) {
  string map_path = "../config/map.csv";
  vector<vector<int>> grid_map;
  load_grid_map(grid_map, map_path);
  AStar astar;
  astar.set_map(grid_map);
  Node start(3, 3);
  Node goal(5, 30);

	bool enable_save = false;
  for(int i = 0; i < argc; i++)
  {
    if (string(argv[i]) == "-start")
    {
      start.x = std::stoi(argv[i+1]);
      start.y = std::stoi(argv[i+2]);
    }
    if (string(argv[i]) == "-goal")
    {
      goal.x = std::stoi(argv[i+1]);
      goal.y = std::stoi(argv[i+2]);
    }
		if (string(argv[i]) == "-save")
		{
			if(argv[i+1] == "true")
				enable_save = true;
			else
				enable_save = false;
		}

  }

  vector<Node> path;
  if(astar.planning(start, goal))
    path = astar.get_path();
  else
    cout << "No path found" << endl;
  
  FILE *gp;
  gp = popen("gnuplot -persist", "w");
	if(enable_save)
	{
  	fprintf(gp, "set terminal png\n");
  	fprintf(gp, "set output 'astar.png'\n");
  }
	fprintf(gp, "set nokey\n");
  fprintf(gp, "set noborder\n");
  fprintf(gp, "set noxtics\n");
  fprintf(gp, "set noytics\n");
  fprintf(gp, "set pm3d map\n");
  fprintf(gp, "unset colorbox\n");
  //fprintf(gp, "set grid\n");
  fprintf(gp, "set size square\n");
  fprintf(gp, "set palette defined (0 'white', 1 'black')\n");
  fprintf(gp, "splot '-' matrix with image, '-' with points, '-' with lines \n"); 
  for(auto row : grid_map) {
    for(auto cell : row)
      fprintf(gp, "%d ", cell);
    fprintf(gp, "\n");
  }
  fprintf(gp, "e\n");
  fprintf(gp, "%d %d 0.5\n", start.x, start.y);
  fprintf(gp, "%d %d 0.5\n", goal.x, goal.y);
  fprintf(gp, "e\n");
  for(auto node : path)
    fprintf(gp, "%d %d 0.5\n", node.x, node.y);
  fprintf(gp, "e\n");
  //fprintf(gp, "set terminal png\n");
  return 0;
}
