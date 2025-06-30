#ifndef PRMSTAR3D_HEADER_INCLUDED
#define PRMSTAR3D_HEADER_INCLUDED

#include <queue>
#include "decomposition_3d.hpp"

class PRMStarPlanner3d {
public:
    Point3f start_pos_;
    Point3f target_pos_;
    std::vector<PolygonObstacle3d> obstacles_;
    Passages3d passages_;
    vector<PolygonCell3d> cells_;
    unordered_map<string, vector<int>> psg_cell_idx_map_;
    unordered_map<int, vector<int>> obs_cell_idx_map_;    
    int planar_cell_num_ = 0;    
    float gamma_prm_star_;
    int cost_function_type_;
    bool plan_in_interior_ = false;
    float interior_diameter_ = 1;
    bool check_all_passages_;
    Size2f config_size_;
    float config_height_;
    PathNode3d* start_node_;
    PathNode3d* target_node_;
    kdTree3d kd_tree_;
    int MAX_GRAPH_SIZE = 2000;
    std::vector<PathNode3d*> node_vec_;
    int GRAPH_SIZE = 0;
    bool plan_success_ = false;
    float min_cost_ = 1e6;

    PRMStarPlanner3d(): start_node_(nullptr), target_node_(nullptr) {}
    PRMStarPlanner3d(Point3f start, Point3f target, vector<PolygonObstacle3d> obs,
                   Size2f config_size = Size2f(640, 480),
                   float config_height = 100, 
                   int cost_function_type = 0,
                   bool check_all_passages = false,
                   bool plan_in_interior = false,
                   float interior_diameter = 1,
                   int max_graph_size = 2000): 
        start_pos_(start), 
        target_pos_(target), 
        obstacles_(obs),
        config_size_(config_size),
        config_height_(config_height),
        cost_function_type_(cost_function_type),
        check_all_passages_(check_all_passages),
        plan_in_interior_(plan_in_interior),
        interior_diameter_(interior_diameter),
        MAX_GRAPH_SIZE(max_graph_size) {
            start_node_ = new PathNode3d(start);
            target_node_ = new PathNode3d(target);
            
            node_vec_ = vector<PathNode3d*>(MAX_GRAPH_SIZE);
            node_vec_[GRAPH_SIZE++] = start_node_;
            gamma_prm_star_ = 2 * cbrt(FreespaceVolume(obstacles_, config_size_, config_height_) / M_PI);

            passages_ = PassageCheckDelaunayGraphWithWalls3d(obstacles_);
            cells_ = GetCompoundGabrielCells3d(obstacles_, passages_);
            start_node_->cell_id = LocatePtInCells(start_node_->pos, cells_);

            for (const PolygonCell3d& cell : cells_) {
                if (!cell.is_an_obstacle)
                    planar_cell_num_++;
            }
        
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

            if (cost_function_type_ < 0 || cost_function_type_ > 7) {
                cost_function_type_ = 0;
            }
            /* std::cout << "RRT* path planner instanced with cost function type: " << cost_function_type_ 
                    << "\n0: Any invalid type value: Default path length cost"
                    << "\n1: Clearance cost: -path clearance"
                    << "\n2: Minimum passage width cost: -min_passed_passage_width"
                    << "\n3: Top-k minimum passage width cost: -k_weight * k_min_passed_passage_widths"
                    << "\n5: Compound cost: len - weight * min_passed_passage_width"
                    << "\n6: Compound cost: len - k_weigth * k_min_passed_passage_width"
                    << "\n7: Ratio cost: len / passed_min_passage_width\n\n"; */  
    }
    ~PRMStarPlanner3d();
    PRMStarPlanner3d(const PRMStarPlanner3d&);
    PRMStarPlanner3d& operator=(const PRMStarPlanner3d&);
    
    void ConstructRoadmap();
    void QueryPath();
    vector<float> QueryCostLog(vector<int> query_samples);
    PathNode3d* FindParentByCost(vector<PathNode3d*> node_vec, PathNode3d* new_node);
    void UpdateNodeCost(PathNode3d* node);
    float NewCost(PathNode3d* near_node, PathNode3d* new_node);
    void UpdateSubtree(PathNode3d* new_parent, PathNode3d* child);
    bool EdgeObstacleFree(PathNode3d* near_node, PathNode3d* new_node);
    vector<PathNode3d*> GetPath();
    vector<Point3f> GetPathInPts();
};

void PRMStarPlanner3d::ConstructRoadmap() {
    random_device rd_x, rd_y, rd_z;
    mt19937 rd_engine_x(rd_x()), rd_engine_y(rd_y()), rd_engine_z(rd_z());
    uniform_real_distribution<> distribution_x(0, config_size_.width), distribution_y(0, config_size_.height), distribution_z(0, config_height_);

    Point3f rand_pos(0, 0, 0);
    float node_clearance = 10000;
    while (GRAPH_SIZE < MAX_GRAPH_SIZE) {
        rand_pos.x = distribution_x(rd_x);
        rand_pos.y = distribution_y(rd_y);
        rand_pos.z = distribution_z(rd_z);
        if (!PointObstacleFree(rand_pos, obstacles_))
            continue;
        if ((cost_function_type_ == 0 && plan_in_interior_) || cost_function_type_ == 1) {
            node_clearance = MinDistanceToObstaclesVec(obstacles_, rand_pos);
            if (cost_function_type_ == 0 && plan_in_interior_ && node_clearance < interior_diameter_) 
                continue; 
        }
        
        PathNode3d* new_node = new PathNode3d(rand_pos);
        new_node->cost = 1e8;
        new_node->len = 1e8;
        new_node->id = GRAPH_SIZE;
        node_vec_[GRAPH_SIZE++] = new_node;
        if (cost_function_type_ == 1)
            new_node->clearance = node_clearance;
    }

    for (auto node : node_vec_)
        kd_tree_.Add(node);
    float radius_alg = gamma_prm_star_ * cbrt(log(MAX_GRAPH_SIZE) / MAX_GRAPH_SIZE);
    for (auto node : node_vec_) {
        float x_min = std::max((float)0.0, node->pos.x - radius_alg), 
              x_max = std::min(node->pos.x + radius_alg, config_size_.width),
              y_min = std::max((float)0.0, node->pos.y - radius_alg), 
              y_max = std::min(node->pos.y + radius_alg, config_size_.height),
              z_min = std::max((float)0.0, node->pos.z - radius_alg),
              z_max = std::min(node->pos.z + radius_alg, config_height_);  
        vector<PathNode3d*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max, z_min, z_max);
        for (auto near_node : near_set) {
            if (!EdgeObstacleFree(node, near_node) 
                || cv::norm(node->pos - near_node->pos) > radius_alg
                || near_node == node)
                continue;
            node->adjacency_list.push_back(near_node);            
        }                    
    }
}

void PRMStarPlanner3d::QueryPath() {
    if (GRAPH_SIZE < MAX_GRAPH_SIZE)
        ConstructRoadmap();
        
    cout << "Query path in PRM*. Cost type: " << cost_function_type_ << "\n";
    vector<bool> visited(node_vec_.size(), false);
    UpdateNodeCost(start_node_);
    float min_cost = FLT_MAX;
    int min_cost_idx = 0;
    // PathNode3d* target_graph_node = kd_tree_.FindNearestNode(target_node_);
    PathNode3d* target_graph_node = FindParentByCost(node_vec_, target_node_);
    target_node_->parent = target_graph_node;
    priority_queue<PathNode3d*, vector<PathNode3d*>, PathNode3dComparator> min_cost_heap;
    min_cost_heap.push(start_node_);

    plan_success_ = true;
    for (int i = 0; i < node_vec_.size(); i++) {
        while (!min_cost_heap.empty() && visited[min_cost_heap.top()->id]) {
            min_cost_heap.pop();
        }
        if (min_cost_heap.empty()) {
            // An unconnected graph is construct;
            plan_success_ = false;
            break;
        }

        PathNode3d* node = min_cost_heap.top();
        min_cost_heap.pop();
        visited[node->id] = true;
        if (node == target_graph_node) {
            min_cost_ = NewCost(target_graph_node, target_node_);
            break;
        }
        
        for (auto adj_node : node->adjacency_list) {
            if (visited[adj_node->id])
                continue;

            float temp_cost = NewCost(node, adj_node);
            if (temp_cost < adj_node->cost 
                || ((cost_function_type_ == 2 || cost_function_type_ == 3) 
                    && temp_cost < adj_node->cost + 1e-2
                    && node->len + cv::norm(adj_node->pos - node->pos) < adj_node->len)) {
                UpdateSubtree(node, adj_node);
                min_cost_heap.push(adj_node);             
            }
        }
    }
}

vector<float> PRMStarPlanner3d::QueryCostLog(vector<int> query_samples) {
    random_device rd_x, rd_y, rd_z;
    mt19937 rd_engine_x(rd_x()), rd_engine_y(rd_y()), rd_engine_z(rd_z());
    uniform_real_distribution<> distribution_x(0, config_size_.width), distribution_y(0, config_size_.height), distribution_z(0, config_height_);

    Point3f rand_pos(0, 0, 0);
    float node_clearance = 10000;
    MAX_GRAPH_SIZE = query_samples.back();
    GRAPH_SIZE = 0;
    node_vec_.clear();
    node_vec_ = vector<PathNode3d*>(MAX_GRAPH_SIZE);
    node_vec_[GRAPH_SIZE++] = start_node_;
    
    while (GRAPH_SIZE < MAX_GRAPH_SIZE) {
        rand_pos.x = distribution_x(rd_x);
        rand_pos.y = distribution_y(rd_y);
        rand_pos.z = distribution_z(rd_z);
        if (!PointObstacleFree(rand_pos, obstacles_))
            continue;
        if ((cost_function_type_ == 0 && plan_in_interior_) || cost_function_type_ == 1) {
            node_clearance = MinDistanceToObstaclesVec(obstacles_, rand_pos);
            if (cost_function_type_ == 0 && plan_in_interior_ && node_clearance < interior_diameter_) 
                continue; 
        }
        
        PathNode3d* new_node = new PathNode3d(rand_pos);
        new_node->cost = 1e8;
        new_node->len = 1e8;
        new_node->id = GRAPH_SIZE;
        node_vec_[GRAPH_SIZE++] = new_node;
        if (cost_function_type_ == 1)
            new_node->clearance = node_clearance;
    }
    
    vector<float> cost_log(query_samples.size(), 1e6);
    for (int query_idx = 0; query_idx < query_samples.size(); query_idx++) {
        int query_sample_num = query_samples[query_idx];

        int start_node_idx = (query_idx == 0 ? 0 : query_samples[query_idx - 1]);
        for (; start_node_idx < query_sample_num; start_node_idx++) {
            kd_tree_.Add(node_vec_[start_node_idx]);
        }
        
        float radius_alg = gamma_prm_star_ * cbrt(log(query_sample_num) / query_sample_num);
        for (int i = 0; i < query_sample_num; i++) {
            PathNode3d* node = node_vec_[i];
            float x_min = std::max((float)0.0, node->pos.x - radius_alg), 
                x_max = std::min(node->pos.x + radius_alg, config_size_.width),
                y_min = std::max((float)0.0, node->pos.y - radius_alg), 
                y_max = std::min(node->pos.y + radius_alg, config_size_.height),
                z_min = std::max((float)0.0, node->pos.z - radius_alg),
                z_max = std::min(node->pos.z + radius_alg, config_height_);    
            vector<PathNode3d*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max, z_min, z_max);
            for (auto near_node : near_set) {
                if (!EdgeObstacleFree(node, near_node) 
                    || cv::norm(node->pos - near_node->pos) > radius_alg
                    || near_node == node)
                    continue;
                node->adjacency_list.push_back(near_node);               
            }
        }

        vector<PathNode3d*> query_nodes(node_vec_.begin(), node_vec_.begin() + query_sample_num);
        vector<bool> visited(query_sample_num, false);
        UpdateNodeCost(start_node_);
        PathNode3d* target_graph_node = FindParentByCost(query_nodes, target_node_);
        if (!target_graph_node)
            continue;
        cout << query_sample_num << "-" << target_graph_node->id << "\n";
        target_node_->parent = target_graph_node;
        priority_queue<PathNode3d*, vector<PathNode3d*>, PathNode3dComparator> min_cost_heap;
        min_cost_heap.push(start_node_);
    
        for (int i = 0; i < query_sample_num; i++) {
            while (!min_cost_heap.empty() && visited[min_cost_heap.top()->id]) {
                min_cost_heap.pop();
            }
            if (min_cost_heap.empty()) {
                cout << "An unconnected graph is present\n";
                break;
            }
    
            PathNode3d* node = min_cost_heap.top();
            min_cost_heap.pop();
            visited[node->id] = true;
            // cout << node->id << " (" << node->cost << "), " << node->clearance << ", ";
            if (node == target_graph_node) {
                cost_log[query_idx] = NewCost(target_graph_node, target_node_);
                break;
            }
            
            for (auto adj_node : node->adjacency_list) {
                if (visited[adj_node->id])
                    continue;
    
                float temp_cost = NewCost(node, adj_node);
                if (temp_cost < adj_node->cost 
                    || ((cost_function_type_ == 2 || cost_function_type_ == 3) 
                        && temp_cost < adj_node->cost + 1e-2
                        && node->len + cv::norm(adj_node->pos - node->pos) < adj_node->len)) {
                    UpdateSubtree(node, adj_node);
                    min_cost_heap.push(adj_node);             
                }
            }
        }
        for (int j = 0; j < query_sample_num; j++) {
            auto node = node_vec_[j];
            node->cost = 1e8;
            node->len = 1e8;
            node->adjacency_list.clear();
        }
        start_node_->len = 0;
        cout << "\n";
    }  
    return cost_log;      
}

PathNode3d* PRMStarPlanner3d::FindParentByCost(vector<PathNode3d*> node_vec, PathNode3d* new_node) {
    PathNode3d* parent = nullptr;
    float min_cost = FLT_MAX, min_len = FLT_MAX;
    for (PathNode3d* node : node_vec) {
        if (!EdgeObstacleFree(node, new_node))
            continue;

        float cost = FLT_MAX, edge_len = cv::norm(node->pos - new_node->pos);
        if (cost_function_type_ == 0)
            cost = edge_len;
        if (cost_function_type_ == 1)
            cost = -min(node->clearance, new_node->clearance);

        vector<float> passed_width_vec = GetPassedPassageWidthsDG3d(node, new_node, obstacles_, passages_, cells_, 
                                                                    psg_cell_idx_map_, obs_cell_idx_map_, planar_cell_num_, false);
        float min_psg_width = node->min_passage_width;
        std::list<float> psg_list = node->sorted_passage_list;
        if (passed_width_vec.size() > 0) {
            for (float& psg_width : passed_width_vec)
                min_psg_width = std::min(min_psg_width, psg_width);
            InsertIntoSortedList(psg_list, passed_width_vec);                
        }        

        if (cost_function_type_ == 2) {
            cost = -min_psg_width;
        }
        else if (cost_function_type_ == 3) {
            cost = 0;
            vector<float> base{1e4, 1e2, 1};
            auto it = psg_list.begin();
            if (psg_list.size() < 3) {
                int i = 0;
                while (i < psg_list.size()) {
                    cost -= base[i] * (*it);
                    i++; it++;
                }
                while (i < 3) {
                    cost -= base[i] * 1000;
                    i++;
                }
            }
            else {
                for (int i = 0; i < 3; i++, it++) 
                    cost -= base[i] * (*it);
            } 
        }  
        if (cost < min_cost || (cost < min_cost + 1e-3 && edge_len < min_len)) {
            min_cost = cost;
            min_len = edge_len;
            parent = node;
        }
    }
    return parent;
}

void PRMStarPlanner3d::UpdateNodeCost(PathNode3d* node) {
    node->cost = 0;
    if (cost_function_type_ == 0) {
        node->cost = node->len;
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
            for (int i = 0; i < 3; i++, it++)
                node->cost -= base[i] * (*it);
        }         
    }
}

float PRMStarPlanner3d::NewCost(PathNode3d* near_node, PathNode3d* new_node) {
    std::list<float> new_psg_list = near_node->sorted_passage_list;
    float new_len = near_node->len + cv::norm(new_node->pos - near_node->pos), 
            new_min_psg_width = near_node->min_passage_width,
            res = 0;
    if (cost_function_type_ == 0)
        return new_len;
    if (cost_function_type_ == 1)
        return -min(-near_node->cost, new_node->clearance);

    vector<float> passed_width_vec;
    if (check_all_passages_) {
        passed_width_vec = GetPassedPassageWidths3d(near_node, new_node, passages_);
    }
    else { 
        passed_width_vec = GetPassedPassageWidthsDG3d(near_node, new_node, obstacles_, passages_, cells_, 
                                                    psg_cell_idx_map_, obs_cell_idx_map_, planar_cell_num_, false);
    }
    if (passed_width_vec.size() > 0) {
        for (float& psg_width : passed_width_vec)
            new_min_psg_width = std::min(new_min_psg_width, psg_width);
        InsertIntoSortedList(new_psg_list, passed_width_vec);                
    }
    
    if (cost_function_type_ == 2) {
        res = -new_min_psg_width;
    }
    else if (cost_function_type_ == 3) {
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
    }
    return res;      
} 

void PRMStarPlanner3d::UpdateSubtree(PathNode3d* new_parent, PathNode3d* child) {
    child->parent = new_parent;
    child->len = new_parent->len + cv::norm(new_parent->pos - child->pos);

    child->min_passage_width = new_parent->min_passage_width;
    child->cur_passage_widths.clear();
    child->sorted_passage_list = new_parent->sorted_passage_list;

    vector<float> passed_width_vec;
    if (cost_function_type_ > 1) {
        if (check_all_passages_) {
            passed_width_vec = GetPassedPassageWidths3d(new_parent, child, passages_);
        }
        else {
            passed_width_vec = GetPassedPassageWidthsDG3d(new_parent, child, obstacles_, passages_, cells_, 
                                                        psg_cell_idx_map_, obs_cell_idx_map_, planar_cell_num_, true);
        }
        if (passed_width_vec.size() > 0) {
            child->cur_passage_widths = passed_width_vec;
            InsertIntoSortedList(child->sorted_passage_list, passed_width_vec); 
            for (float& psg_width : passed_width_vec)
                child->min_passage_width = std::min(child->min_passage_width, psg_width);
        }      
    }
    UpdateNodeCost(child); 
}

bool PRMStarPlanner3d::EdgeObstacleFree(PathNode3d* near_node, PathNode3d* new_node) {
    for (auto& obs : obstacles_)
        if (!ObstacleFree(obs, near_node->pos, new_node->pos))
            return false;
    return true;
}

vector<PathNode3d*> PRMStarPlanner3d::GetPath() {
    std::vector<PathNode3d*> res;
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

vector<Point3f> PRMStarPlanner3d::GetPathInPts() {
    vector<PathNode3d*> node_path = GetPath();
    vector<Point3f> res(node_path.size());

    for (int i = 0; i < node_path.size(); i++)
        res[i] = node_path[i]->pos;
    return res;
}

PRMStarPlanner3d::~PRMStarPlanner3d() { 
}

PRMStarPlanner3d::PRMStarPlanner3d(const PRMStarPlanner3d& planner) {
      
}

PRMStarPlanner3d& PRMStarPlanner3d::operator=(const PRMStarPlanner3d& rhs) {
    return *this;
}
#endif
