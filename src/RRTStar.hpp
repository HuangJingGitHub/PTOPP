#ifndef RRTSTAR_HEADER_INCLUDED
#define RRTSTAR_HEADER_INCLUDED

#include <queue>
#include "decomposition.hpp"

class RRTStarPlanner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    std::vector<PolygonObstacle> obstacles_;
    std::vector<std::vector<int>> passage_pairs_;
    std::vector<std::vector<Point2f>> passage_pts_;
    std::vector<std::vector<int>> cells_;
    std::vector<PolygonCell> cells_info_;
    std::unordered_map<std::string, std::vector<int>> psg_cell_idx_map_;
    float gamma_rrt_star_;
    float step_len_;
    int cost_function_type_;
    bool is_passage_width_constrained_;
    float passage_width_threshold_;
    bool check_all_passages_;
    Mat source_img_;
    Size2f config_size_;
    PathNode* start_node_;
    PathNode* target_node_;
    kdTree kd_tree_;
    int MAX_GRAPH_SIZE = 5000;
    int GRAPH_SIZE = 0;
    bool plan_success_ = false;
    vector<float> min_cost_log_; 

    RRTStarPlanner(): start_node_(nullptr), target_node_(nullptr) {}
    RRTStarPlanner(Point2f start, Point2f target, vector<PolygonObstacle> obs, 
                   float step_len = 20, 
                   Size2f config_size = Size2f(640, 480), 
                   int cost_function_type = 0,
                   bool check_all_passages = false,
                   bool is_passage_width_constrained = false,
                   float passage_width_threshold = 50): 
        start_pos_(start), 
        target_pos_(target), 
        obstacles_(obs),
        step_len_(step_len),
        config_size_(config_size),
        check_all_passages_(check_all_passages),
        cost_function_type_(cost_function_type),
        is_passage_width_constrained_(is_passage_width_constrained),
        passage_width_threshold_(passage_width_threshold) {
            start_node_ = new PathNode(start);
            target_node_ = new PathNode(target);
            gamma_rrt_star_ = 4 * sqrt(FreespaceArea(obstacles_, config_size_) / M_PI);   

            Passages DG_check_res = PassageCheckDelaunayGraphWithWalls(obstacles_);
            passage_pairs_ = DG_check_res.pairs;
            passage_pts_ = DG_check_res.pts; 

            cells_ = ReportGabrielCells(obstacles_, passage_pairs_, true);
            cells_info_ = GetGabrielCellsInfo(cells_, DG_check_res);
            psg_cell_idx_map_ = GetPassageCellMap(cells_info_);
            start_node_->cell_id = LocatePtInCells(start_node_->pos, cells_info_);
            
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
    ~RRTStarPlanner();
    RRTStarPlanner(const RRTStarPlanner&);
    RRTStarPlanner& operator=(const RRTStarPlanner&);
    bool Plan(Mat source_img, int max_graph_size = 5000, bool plan_in_interior = false, float interior_diameter = 0.5);
    PathNode* GenerateNewNode(PathNode* nearest_node, Point2f& rand_pos);
    bool EdgeObstacleFree(PathNode* near_node, PathNode* new_node);
    void Rewire(PathNode* nearest_node, PathNode* new_node, Mat source_img);
    float NewCost(PathNode* near_node, PathNode* new_node);
    void UpdateNodeCost(PathNode* node);
    void UpdateSubtree(PathNode* new_parent, PathNode* child);
    Point2f SafeRandTarget();
    vector<PathNode*> GetPath();
    vector<Point2f> GetPathInPts();
};

#endif
