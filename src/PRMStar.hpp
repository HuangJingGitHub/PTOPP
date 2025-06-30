#ifndef PRMSTAR_HEADER_INCLUDED
#define PRMSTAR_HEADER_INCLUDED

/* #include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp> */
#include <queue>
#include "decomposition.hpp"

class PRMStarPlanner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    std::vector<PolygonObstacle> obstacles_;
    std::vector<std::vector<int>> ev_passage_pairs_;
    std::vector<std::vector<Point2f>> ev_passage_pts_;
    std::vector<std::vector<int>> cells_;
    std::vector<PolygonCell> cells_info_;
    std::unordered_map<std::string, std::vector<int>> psg_cell_idx_map_;
    float gamma_prm_star_;
    int cost_function_type_;
    bool plan_in_interior_ = false;
    float interior_diameter_ = 1;
    bool check_all_passages_;
    Size2f config_size_;
    PathNode* start_node_;
    PathNode* target_node_;
    kdTree kd_tree_;
    int MAX_GRAPH_SIZE = 2000;
    std::vector<PathNode*> node_vec_;
    int GRAPH_SIZE = 0;
    bool plan_success_ = false;
    float min_cost_ = 1e6;

    PRMStarPlanner(): start_node_(nullptr), target_node_(nullptr) {}
    PRMStarPlanner(Point2f start, Point2f target, vector<PolygonObstacle> obs,
                   Size2f config_size = Size2f(640, 480), 
                   int cost_function_type = 0,
                   bool check_all_passages = false,
                   bool plan_in_interior = false,
                   float interior_diameter = 1,
                   int max_graph_size = 2000): 
        start_pos_(start), 
        target_pos_(target), 
        obstacles_(obs),
        config_size_(config_size),
        cost_function_type_(cost_function_type),
        check_all_passages_(check_all_passages),
        plan_in_interior_(plan_in_interior),
        interior_diameter_(interior_diameter),
        MAX_GRAPH_SIZE(max_graph_size) {
            start_node_ = new PathNode(start);
            target_node_ = new PathNode(target);
            
            node_vec_ = vector<PathNode*>(MAX_GRAPH_SIZE);
            node_vec_[GRAPH_SIZE++] = start_node_;
            gamma_prm_star_ = sqrt(6 * FreespaceArea(obstacles_, config_size_) / M_PI);

            Passages DG_check_res = PassageCheckDelaunayGraphWithWalls(obstacles_);
            ev_passage_pairs_ = DG_check_res.pairs;
            ev_passage_pts_ = DG_check_res.pts; 

            cells_ = ReportGabrielCells(obstacles_, ev_passage_pairs_, true);
            cells_info_ = GetGabrielCellsInfo(cells_, DG_check_res);
            psg_cell_idx_map_ = GetPassageCellMap(cells_info_);
            start_node_->cell_id = LocatePtInCells(start_node_->pos, cells_info_);

            if (cost_function_type_ < 0 || cost_function_type_ > 6) {
                cost_function_type_ = 0;
            }
            /* std::cout << "PRM* path planner instanced with cost function type: " << cost_function_type_ 
                    << "\n0: Any invalid type value: Default path length cost"
                    << "\n1: Clearance cost: -path clearance"
                    << "\n2: Minimum passage width cost: -min_passed_passage_width"
                    << "\n3: Top-k minimum passage width cost: -k_weight * k_min_passed_passage_widths"
                    << "\n5: Compound cost: len - weight * min_passed_passage_width"
                    << "\n6: Compound cost: len - k_weigth * k_min_passed_passage_width"
                    << "\n7: Ratio cost: len / passed_min_passage_width\n\n"; */ 
    }
    ~PRMStarPlanner();
    PRMStarPlanner(const PRMStarPlanner&);
    PRMStarPlanner& operator=(const PRMStarPlanner&);
    
    void ConstructRoadmap(Mat source_img);
    void QueryPath(Mat source_img);
    vector<float> QueryCostLog(vector<int> query_samples);
    PathNode* FindParentByCost(vector<PathNode*> node_vec, PathNode* new_node);
    void UpdateNodeCost(PathNode* node);
    float NewCost(PathNode* near_node, PathNode* new_node);
    void UpdateSubtree(PathNode* new_parent, PathNode* child);
    bool EdgeObstacleFree(PathNode* near_node, PathNode* new_node);
    vector<PathNode*> GetPath();
};
#endif
