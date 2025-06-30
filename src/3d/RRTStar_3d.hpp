#ifndef RRTSTAR3D_HEADER_INCLUDED
#define RRTSTAR3D_HEADER_INCLUDED

#include <queue>
#include "decomposition_3d.hpp"
// #include "obstacles_3d.hpp"
// #include "kd_tree_3d.hpp"

class RRTStarPlanner3d {
public:
    Point3f start_pos_;
    Point3f target_pos_;
    std::vector<PolygonObstacle3d> obstacles_;
    Passages3d passages_;
    vector<PolygonCell3d> cells_;
    unordered_map<string, vector<int>> psg_cell_idx_map_;
    unordered_map<int, vector<int>> obs_cell_idx_map_;    
    int planar_cell_num_ = 0;
    float gamma_rrt_star_;
    float step_len_;
    int cost_function_type_;
    bool is_passage_width_constrained_;
    float passage_width_threshold_;
    bool check_all_passages_;
    Size2f config_size_;
    float config_height_;
    PathNode3d* start_node_;
    PathNode3d* target_node_;
    kdTree3d kd_tree_;
    int MAX_GRAPH_SIZE = 2000;
    int GRAPH_SIZE = 0;
    bool plan_success_ = false;
    vector<float> min_cost_log_; 

    RRTStarPlanner3d(): start_node_(nullptr), target_node_(nullptr) {}
    RRTStarPlanner3d(Point3f start, Point3f target, vector<PolygonObstacle3d> obs, 
                   float step_len = 20, 
                   Size2f config_size = Size2f(640, 480), 
                   float config_height = 100,
                   int cost_function_type = 0,
                   bool check_all_passages = false,
                   bool is_passage_width_constrained = false,
                   float passage_width_threshold = 100): 
        start_pos_(start), 
        target_pos_(target), 
        obstacles_(obs),
        step_len_(step_len),
        config_size_(config_size),
        config_height_(config_height),
        cost_function_type_(cost_function_type),
        check_all_passages_(check_all_passages),
        is_passage_width_constrained_(is_passage_width_constrained),
        passage_width_threshold_(passage_width_threshold) {
            start_node_ = new PathNode3d(start);
            target_node_ = new PathNode3d(target);
            gamma_rrt_star_ = 4 * cbrt(FreespaceVolume(obstacles_, config_size_, config_height_) * 3 / 4 / M_PI);  
            
            passages_ = PassageCheckDelaunayGraphWithWalls3d(obstacles_);
            cells_ = GetCompoundGabrielCells3d(obstacles_, passages_);
            start_node_->cell_id = LocatePtInCells(start_node_->pos, cells_);

            for (const PolygonCell3d& cell : cells_) 
                if (!cell.is_an_obstacle)
                    planar_cell_num_++;
        
            for (int i = 0; i < cells_.size(); i++) {
                if (cells_[i].is_an_obstacle)
                    continue;
        
                int vertex_num = cells_[i].obs_indices.size();
                for (int j = 0; j < vertex_num; j++) {
                    string key_str = PassagePairToKeyString(cells_[i].obs_indices[j], cells_[i].obs_indices[(j + 1) % vertex_num]);
                    psg_cell_idx_map_[key_str].push_back(i);
                    obs_cell_idx_map_[cells_[i].obs_indices[j]].push_back(i);
                }
            }            

            if (cost_function_type_ < 0 || cost_function_type_ > 6) {
                cost_function_type_ = 0;
            }
            UpdateNodeCost(start_node_);
            kd_tree_.Add(start_node_);
            GRAPH_SIZE++;

            /* std::cout << "RRT* path planner instanced with cost function type: " << cost_function_type_ 
                    << "\n0: Any invalid type value: Default path length cost"
                    << "\n1: Clearance cost: -path clearance"
                    << "\n2: Minimum passage width cost: -min_passed_passage_width"
                    << "\n3: Top-k minimum passage width cost: -k_weight * k_min_passed_passage_widths"
                    << "\n5: Compound cost: len - weight * min_passed_passage_width"
                    << "\n6: Compound cost: len - k_weigth * k_min_passed_passage_width"
                    << "\n7: Ratio cost: len / passed_min_passage_width\n\n"; */    
    }
    ~RRTStarPlanner3d();
    RRTStarPlanner3d(const RRTStarPlanner3d&);
    RRTStarPlanner3d& operator=(const RRTStarPlanner3d&);
    bool Plan(int max_graph_size = 2000, bool plan_in_interior = false, float interior_diameter = 0.5);
    PathNode3d* GenerateNewNode(PathNode3d* nearest_node, Point3f& rand_pos);
    bool EdgeObstacleFree(PathNode3d* near_node, PathNode3d* new_node);
    void Rewire(PathNode3d* nearest_node, PathNode3d* new_node);
    float NewCost(PathNode3d* near_node, PathNode3d* new_node);
    void UpdateNodeCost(PathNode3d* node);
    void UpdateSubtree(PathNode3d* new_parent, PathNode3d* child);
    Point3f SafeRandTarget();
    vector<PathNode3d*> GetPath();
    vector<Point3f> GetPathInPts();
};

#endif
