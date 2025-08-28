#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>
#include <algorithm> 
using namespace std;

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  vector<pair<int, int>> path; // store final path 
			       
  /* Implement Path Planning logic here */
  
// Directions: up, down, left, right
  vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1}};

  using Node = pair<double, pair<int,int>>; // f-score, (x,y)
  priority_queue<Node, vector<Node>, greater<Node>> openSet;

  unordered_map<long long, double> gScore;
  unordered_map<long long, pair<int,int>> cameFrom;

  auto key = [this](int x, int y){ return ((long long)x<<32)|(unsigned int)y; };

  gScore[key(start.first, start.second)] = 0.0;
  openSet.push({heuristic(start.first, start.second, goal.first, goal.second), start});

  while(!openSet.empty()){
    auto [f, current] = openSet.top(); openSet.pop();
    int cx = current.first, cy = current.second;

    // Goal reached → reconstruct path
    if(current == goal){
      pair<int,int> node = goal;
      while(node != start){
        path.push_back(node);
        node = cameFrom[key(node.first, node.second)];
      }
      path.push_back(start);
      reverse(path.begin(), path.end());
      return path;
    }

    // Explore neighbors
    for(auto [dx,dy] : directions){
      int nx = cx+dx, ny = cy+dy;
      if(!isvalid(nx,ny)) continue;

      double tentative_g = gScore[key(cx,cy)] + 1.0;
      auto nk = key(nx,ny);

      if(!gScore.count(nk) || tentative_g < gScore[nk]){
        cameFrom[nk] = {cx,cy};
        gScore[nk] = tentative_g;
        double fscore = tentative_g + heuristic(nx,ny,goal.first,goal.second);
        openSet.push({fscore,{nx,ny}});
      }
    }
  }

  return path;
}
