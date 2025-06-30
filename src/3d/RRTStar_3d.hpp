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

bool RRTStarPlanner3d::Plan(int max_graph_size, bool plan_in_interior, float interior_diameter) {       
    plan_success_ = false;
    float   min_cost = FLT_MAX,
            total_cost = FLT_MAX,
            min_len = FLT_MAX,
            total_len = FLT_MAX;

    random_device rd_x, rd_y, rd_z;
    mt19937 rd_x_engine(rd_x()), rd_y_engine(rd_y()), rd_z_engine(rd_z());
    uniform_real_distribution<> distr_x(0, config_size_.width), distr_y(0, config_size_.height), distr_z(0, config_height_);

    srand(time(NULL));  // for SafeRandTarget().
    float dist_to_goal = 50;
    Point3f rand_pos = Point3f(0, 0, 0);
    int sample_num = 0;
    MAX_GRAPH_SIZE = max_graph_size;
    min_cost_log_ = vector<float>(MAX_GRAPH_SIZE, 1e6);
    while (GRAPH_SIZE < MAX_GRAPH_SIZE) {
        // Bias to target
        sample_num++;
        if (sample_num % (MAX_GRAPH_SIZE / 20) == 0) {
            rand_pos = SafeRandTarget();                         
        } 
        else {
            rand_pos.x = distr_x(rd_x);
            rand_pos.y = distr_y(rd_y);
            rand_pos.z = distr_z(rd_z);
        }

        PathNode3d* nearest_node = kd_tree_.FindNearestNode(rand_pos);
        PathNode3d* new_node = GenerateNewNode(nearest_node, rand_pos);

        if (cost_function_type_ == 0 && plan_in_interior 
            && MinDistanceToObstaclesVec(obstacles_, rand_pos) < interior_diameter) {
            delete new_node;
            continue;
        }

        if (EdgeObstacleFree(nearest_node, new_node)) {
            Rewire(nearest_node, new_node);
            kd_tree_.Add(new_node);
            if (SquaredNorm(new_node->pos - target_pos_) <= dist_to_goal* dist_to_goal
                && EdgeObstacleFree(new_node, target_node_)) {
                total_cost = NewCost(new_node, target_node_);
                total_len = new_node->len + cv::norm(new_node->pos - target_node_->pos);
                if (total_cost < min_cost) {
                    target_node_->parent = new_node;
                    min_cost = total_cost;
                    min_len = total_len;
                }      
                plan_success_ = true;
            }
            GRAPH_SIZE++;
            min_cost_log_[GRAPH_SIZE] = std::min(min_cost, (float)1e6);
            // cout << GRAPH_SIZE << "\n";
        }
        else {
            delete new_node;
        }
    }

    if (plan_success_ == false)
        std::cout << "MAX_GRAPH_SIZE: " << MAX_GRAPH_SIZE << " is achieved with no path found.\n";
    else
        std::cout << "Path found with cost: " << min_cost
                << "\nTotal sample number: " << sample_num
                << "\n";   
    return plan_success_;
} 

PathNode3d* RRTStarPlanner3d::GenerateNewNode(PathNode3d* nearest_node, Point3f& rand_pos) {
    float dist = cv::norm(rand_pos - nearest_node->pos);
    Point3f direction = (rand_pos - nearest_node->pos) / dist, new_pos;
    if (dist > step_len_)
        new_pos = nearest_node->pos + step_len_ * direction;
    else
        new_pos = rand_pos;
    new_pos.x = std::max((float)0.5, new_pos.x);
    new_pos.x = std::min(config_size_.width - (float)0.5, new_pos.x);
    new_pos.y = std::max((float)0.5, new_pos.y);
    new_pos.y = std::min(config_size_.height - (float)0.5, new_pos.y);
    new_pos.z = std::max((float)0.5, new_pos.z);
    new_pos.z = std::min(config_height_ - (float)0.5, new_pos.z);    
    PathNode3d* new_node = new PathNode3d(new_pos);
    new_node->id = GRAPH_SIZE;
    if (cost_function_type_ == 1)
        new_node->clearance = MinDistanceToObstaclesVec(obstacles_, new_node->pos);
    return new_node;
}

bool RRTStarPlanner3d::EdgeObstacleFree(PathNode3d* near_node, PathNode3d* new_node) {
    for (auto& obs : obstacles_)
        if (!ObstacleFree(obs, near_node->pos, new_node->pos))
            return false;
    return true;
}

void RRTStarPlanner3d::Rewire(PathNode3d* nearest_node, PathNode3d* new_node) {
    float gamma = gamma_rrt_star_ * cbrt(log(GRAPH_SIZE) / GRAPH_SIZE),
          radius_alg = std::min(gamma, step_len_);
    float x_min = std::max((float)0.0, new_node->pos.x - radius_alg), 
          x_max = std::min(new_node->pos.x + radius_alg, config_size_.width),
          y_min = std::max((float)0.0, new_node->pos.y - radius_alg), 
          y_max = std::min(new_node->pos.y + radius_alg, config_size_.height),
          z_min = std::max((float)0.0, new_node->pos.z - radius_alg),
          z_max = std::min(new_node->pos.z + radius_alg, config_height_); 

    std::vector<PathNode3d*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max, z_min, z_max);
    int k = 0;
    for (int i = 0; i < near_set.size(); i++)
        if (EdgeObstacleFree(near_set[i], new_node) && cv::norm(near_set[i]->pos - new_node->pos) <= radius_alg) {
            near_set[k++] = near_set[i];
        }
    near_set.resize(k);

    // find minimum-cost path
    PathNode3d* min_cost_node = nearest_node;
    float min_cost = NewCost(nearest_node, new_node);
    for (auto near_node : near_set) {
        float cur_cost = NewCost(near_node, new_node);
        if (cur_cost < min_cost) {
            min_cost_node = near_node;
            min_cost = cur_cost;
        }            
    }
    UpdateSubtree(min_cost_node, new_node); 

    for (auto near_node : near_set) {
        if (near_node == min_cost_node)
            continue;
        float new_near_node_cost = NewCost(new_node, near_node);
        if (new_near_node_cost < near_node->cost 
            || (new_near_node_cost <= near_node->cost + 1e-2
                && new_node->len + cv::norm(near_node->pos - new_node->pos) < near_node->len)) {
            UpdateSubtree(new_node, near_node);
        }
    }
}

float RRTStarPlanner3d::NewCost(PathNode3d* near_node, PathNode3d* new_node) {
    std::list<float> new_psg_list = near_node->sorted_passage_list;
    float new_len = near_node->len + cv::norm(new_node->pos - near_node->pos), 
          new_min_psg_width = near_node->min_passage_width,
          res = 0;
    if (cost_function_type_ == 0 && !is_passage_width_constrained_)
        return new_len;            

    if (cost_function_type_ == 1) {
        return -min(-near_node->cost, new_node->clearance);
    } 

    int cnt = 0;
    vector<float> passed_width_vec;
    if (check_all_passages_) {
        passed_width_vec = GetPassedPassageWidths3d(near_node, new_node, passages_);
    }
    else {
        passed_width_vec = GetPassedPassageWidthsDG3d(near_node, new_node, obstacles_, passages_, cells_, 
                                                        psg_cell_idx_map_, obs_cell_idx_map_, planar_cell_num_, false);
    }
    if (passed_width_vec.size()) {
        for (float& width : passed_width_vec) {
            if (is_passage_width_constrained_ && width <= passage_width_threshold_) {
                width = -10000 / width;
                cnt++;
            }
            new_min_psg_width = min(new_min_psg_width, width);
        }
        InsertIntoSortedList(new_psg_list, passed_width_vec);
    }

    if (cost_function_type_ == 0) {
            return near_node->cost + cv::norm(new_node->pos - near_node->pos) + cnt * 1000;
    }
    if (cost_function_type_ == 2) {
        return -new_min_psg_width;
    }
    if (cost_function_type_ == 3) {
        vector<float> base{1e4, 1e2, 1};
        auto it = new_psg_list.begin();
        if (new_psg_list.size() < 3) {
            int i = 0;
            while (i < new_psg_list.size()) {
                res -= base[i] * (*it);
                i++; it++;
            }
            while (i < 3) {
                res -= base[i] * 1000;
                i++;
            }
        }
        else {
            for (int i = 0; i < 3; i++, it++) 
                res -= base[i] * (*it);
        } 
        return res;
    }
    return res;
}    

void RRTStarPlanner3d::UpdateNodeCost(PathNode3d* node) {
    node->cost = 0;
    if (cost_function_type_ == 0 && !is_passage_width_constrained_) {
        node->cost = node->len;
    }            
    else if (cost_function_type_ == 0 && is_passage_width_constrained_) {
        if (node->parent == nullptr) {
            node->cost = node->len;
            return;
        }
        node->cost = node->parent->cost + (node->len - node->parent->len);
        if (node->cur_passage_widths.size() > 0) {
            for (float width : node->cur_passage_widths) {
                if (width <= passage_width_threshold_) {
                    node->cost += 1000;
                }
            }
        }
    }
    else if (cost_function_type_ == 1) {
        if (node->parent == nullptr)
            node->cost = -1000;
        else
            node->cost = -min(node->clearance, -node->parent->cost);        
    }
    else if (cost_function_type_ == 2) {
        node->cost = -node->min_passage_width;
    }            
    else if (cost_function_type_ == 3) {
        vector<float> base{1e4, 1e2, 1};
        auto it = node->sorted_passage_list.begin();
        if (node->sorted_passage_list.size() < 3) {
            int i = 0;
            while (i < node->sorted_passage_list.size()) {
                node->cost -= base[i] * (*it);
                i++; it++;
            }
            while (i < 3) {
                node->cost -= base[i] * 1000;
                i++;
            }
        }
        else {
            for (int i = 0; i < 3; i++, it++) {
                node->cost -= base[i] * (*it);
            }
        } 
    }
}

void RRTStarPlanner3d::UpdateSubtree(PathNode3d* new_parent, PathNode3d* child) {
    PathNode3d* old_parent = child->parent;
    if (old_parent != nullptr) 
        old_parent->children.remove(child);
    child->parent = new_parent;
    new_parent->children.push_back(child);

    float old_len = child->len, len_change = 0;
    child->len = new_parent->len + cv::norm(new_parent->pos - child->pos);
    len_change = child->len - old_len;

    child->min_passage_width = new_parent->min_passage_width;
    child->cur_passage_widths.clear();  
    child->sorted_passage_list = new_parent->sorted_passage_list;
    
    vector<float> passed_width_vec;
    if (passed_width_vec.size() > 1) {   
        if (check_all_passages_) {
            passed_width_vec = GetPassedPassageWidths3d(new_parent, child, passages_);
        }
        else {
            passed_width_vec = GetPassedPassageWidthsDG3d(new_parent, child, obstacles_, passages_, cells_,
                                                        psg_cell_idx_map_, obs_cell_idx_map_, planar_cell_num_, true);
        }

        for (float& psg_width : passed_width_vec) {
            if (is_passage_width_constrained_ && psg_width <= passage_width_threshold_)
                psg_width = -10000 / psg_width;
        }            
        child->cur_passage_widths = passed_width_vec;
        InsertIntoSortedList(child->sorted_passage_list, passed_width_vec); 
        for (float& psg_width : passed_width_vec)
            child->min_passage_width = std::min(child->min_passage_width, psg_width);
    }   
    UpdateNodeCost(child); 

    std::queue<PathNode3d*> node_level;
    node_level.push(child);
    int loop_num = 0;
    while (node_level.empty() == false) {
        PathNode3d* cur_node = node_level.front();
        node_level.pop();

        loop_num++;
        if (loop_num > 1 && cur_node->id == child->id) {
            throw std::invalid_argument("Circle detected in the tree");
        }
        
        for (auto& cur_child : cur_node->children) {
            cur_child->len += len_change;
            cur_child->min_passage_width = cur_node->min_passage_width;
            cur_child->sorted_passage_list = cur_node->sorted_passage_list;
            if (cur_child->cur_passage_widths.size() > 0) {
                for (float& psg_width : cur_child->cur_passage_widths) {
                    cur_child->min_passage_width = std::min(cur_node->min_passage_width, psg_width);
                }
                InsertIntoSortedList(cur_child->sorted_passage_list, cur_child->cur_passage_widths);
            }            
            UpdateNodeCost(cur_child);
            node_level.push(cur_child);
        }
    }
}

/// Instead of directly using rand_pos = target_pos_, add a small random shift to avoid resulting in
/// samples in the same position and potentially any further problematic circle connection in the graph.
Point3f RRTStarPlanner3d::SafeRandTarget() {
    float   safe_dist_x = min(target_pos_.x, config_size_.width - target_pos_.x),
            safe_dist_y = min(target_pos_.y, config_size_.height - target_pos_.y),
            safe_dist_z = min(target_pos_.z, config_height_ - target_pos_.z),
            safe_dist = min(min(safe_dist_x, safe_dist_y), safe_dist_z) * 0.95;
    float r = sqrt(rand() / (float)RAND_MAX) * safe_dist;
    Point3f direction;
    direction.x = rand() / (float)RAND_MAX; direction.y = rand() / (float)RAND_MAX; direction.z = rand() / (float)RAND_MAX;
    direction = direction - Point3f(0.5, 0.5, 0.5);
    direction = direction / cv::norm(direction);
    return target_pos_ + direction * r;
}

vector<PathNode3d*> RRTStarPlanner3d::GetPath() {
    vector<PathNode3d*> res;
    if (!plan_success_) {
        std::cout << "No valid path is available.\n";
        return res;
    }
    PathNode3d* reverse_node = target_node_;
    while (reverse_node) {
        res.push_back(reverse_node);
        reverse_node = reverse_node->parent;
    }
    reverse(res.begin(), res.end());
    return res;   
}  

vector<Point3f> RRTStarPlanner3d::GetPathInPts() {
    vector<PathNode3d*> node_path = GetPath();
    vector<Point3f> res(node_path.size());

    for (int i = 0; i < node_path.size(); i++)
        res[i] = node_path[i]->pos;
    return res;
}

RRTStarPlanner3d::~RRTStarPlanner3d() {
    // delete start_node_;
    // delete target_node_;
}

RRTStarPlanner3d::RRTStarPlanner3d(const RRTStarPlanner3d& planner) {
    start_pos_ = planner.start_pos_;
    target_pos_ = planner.target_pos_;
    if (!start_node_)
        start_node_ = new PathNode3d(start_pos_);
    else {
        kd_tree_.deleteTree(start_node_);
        start_node_ = new PathNode3d(start_pos_);
        kd_tree_.kd_tree_root_ = start_node_;
    }
    if (!target_node_)
        target_node_ = new PathNode3d(target_pos_);
    else {
        delete target_node_;
        target_node_ = new PathNode3d(target_pos_);
    }
    obstacles_ = planner.obstacles_;
    step_len_ = planner.step_len_;
    config_size_ = planner.config_size_;
    MAX_GRAPH_SIZE = planner.MAX_GRAPH_SIZE;
    GRAPH_SIZE = planner.GRAPH_SIZE;
    plan_success_ = planner.plan_success_;        
}

RRTStarPlanner3d& RRTStarPlanner3d::operator=(const RRTStarPlanner3d& rhs) {
    start_pos_ = rhs.start_pos_;
    target_pos_ = rhs.target_pos_;
    if (!start_node_)
        start_node_ = new PathNode3d(start_pos_);
    else {
        kd_tree_.deleteTree(start_node_);
        start_node_ = new PathNode3d(start_pos_);
        kd_tree_.kd_tree_root_ = start_node_;
    }
    if (!target_node_)
        target_node_ = new PathNode3d(target_pos_);
    else {
        delete target_node_;
        target_node_ = new PathNode3d(target_pos_);
    }
    
    obstacles_ = rhs.obstacles_;
    step_len_ = rhs.step_len_;
    config_size_ = rhs.config_size_;    
    MAX_GRAPH_SIZE = rhs.MAX_GRAPH_SIZE;
    GRAPH_SIZE = rhs.GRAPH_SIZE;
    plan_success_ = rhs.plan_success_;
    return *this;
}
#endif