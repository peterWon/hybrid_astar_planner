#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <vector>
#include <map>
#include <stdint.h>

struct Cell{
public:
    Cell(int r,int c):_row(r),_col(c){}
    int _row;
    int _col;
};

struct CellWithDist: public Cell{
public:
    CellWithDist(int r, int c, int d): Cell(r, c),_dist_from_obstacle(d){}
    int _dist_from_obstacle;
};

struct SearchPoint{
public:
    SearchPoint(int row, int col, double c, double h):
        _row(row),_col(col),_cost_so_far(c),_priority(h){}
    int _row;
    int _col;
    double _cost_so_far;
    double _priority;
};

class AStar{
public:
    AStar(){}
    ~AStar();
    bool findPath(const Cell& start,
                  const Cell& goal,
                  std::vector<Cell>& path);
    void buildGraph(const uint8_t* cdata,
                    const int grid_width,
                    const int grid_height,
                    const int inflation_cell_size);
    
    void buildHeuristicGraph(const uint8_t* cdata,
                    const Cell& start,
                    const Cell& goal,
                    const int grid_width,
                    const int grid_height,
                    const int inflation_cell_size);
    uint8_t getCellCost(int row, int col){
        if(row < 0 || col<0 || row>=_height || col>>_width) return 255;
        return _heuristic_graph[row*_width+col];
    }
private:
    uint8_t getCost(int distance);
    uint8_t getHeuristicCost(int distance);
    void appendCells(int r, int c, int distance);
    void appendHeuristicCells(int r, int c, int distance);
    std::vector<uint8_t> _graph;
    std::vector<uint8_t> _heuristic_graph;
    std::vector<bool> _visited;
    std::map<int, std::vector<CellWithDist> > _cells_cache;
    
    int _width;
    int _height;
    int _inflation_radius;
    const double _cost_descend_rate = 0.5;
    const uint8_t _grid_lethal_obstacle = 254;
    const uint8_t _grid_inflation_radius = 253;
};


#endif