#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

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

  FILE *gp;
  gp = popen("gnuplot -persist", "w");
  fprintf(gp, "set nokey\n");
  fprintf(gp, "set noborder\n");
  fprintf(gp, "set noxtics\n");
  fprintf(gp, "set noytics\n");
  fprintf(gp, "set pm3d map\n");
  fprintf(gp, "unset colorbox\n");
  //fprintf(gp, "set grid\n");
  fprintf(gp, "set size square\n");
  fprintf(gp, "set palette defined (0 'white', 1 'black')\n");
  fprintf(gp, "splot '-' matrix with image\n"); 
  for(auto row : grid_map) {
    for(auto cell : row)
      fprintf(gp, "%d ", cell);
    fprintf(gp, "\n");
  }
  fprintf(gp, "e\n");
  //fprintf(gp, "set terminal png\n");
  return 0;
}