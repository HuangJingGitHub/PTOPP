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

void PRMStarPlanner::ConstructRoadmap(Mat source_img) {
    random_device rd_x, rd_y;
    mt19937 rd_engine_x(rd_x()), rd_engine_y(rd_y());
    uniform_real_distribution<> distribution_x(0, config_size_.width), distribution_y(0, config_size_.height);

    Point2f rand_pos(0, 0);
    float node_clearance = 10000;
    while (GRAPH_SIZE < MAX_GRAPH_SIZE) {
        rand_pos.x = distribution_x(rd_x);
        rand_pos.y = distribution_y(rd_y);
        if (!PointObstacleFree(rand_pos, obstacles_))
            continue;
        if ((cost_function_type_ == 0 && plan_in_interior_) || cost_function_type_ == 1) {
            node_clearance = MinDistanceToObstaclesVec(obstacles_, rand_pos);
            if (cost_function_type_ == 0 && plan_in_interior_ && node_clearance < interior_diameter_) 
                continue; 
        }
        
        PathNode* new_node = new PathNode(rand_pos);
        new_node->cost = 1e8;
        new_node->len = 1e8;
        new_node->id = GRAPH_SIZE;
        node_vec_[GRAPH_SIZE++] = new_node;
        if (cost_function_type_ == 1)
            new_node->clearance = node_clearance;
        // circle(source_img, rand_pos, 3, Scalar(0, 255, 0), -1);
    }

    for (auto node : node_vec_)
        kd_tree_.Add(node);
    float radius_alg = gamma_prm_star_ * sqrt(log(MAX_GRAPH_SIZE) / MAX_GRAPH_SIZE);
    for (auto node : node_vec_) {
        float x_min = std::max((float)0.0, node->pos.x - radius_alg), 
              x_max = std::min(node->pos.x + radius_alg, config_size_.width),
              y_min = std::max((float)0.0, node->pos.y - radius_alg), 
              y_max = std::min(node->pos.y + radius_alg, config_size_.height);  
        vector<PathNode*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max);
        for (auto near_node : near_set) {
            if (!EdgeObstacleFree(node, near_node) 
                || cv::norm(node->pos - near_node->pos) > radius_alg
                || near_node == node)
                continue;
            node->adjacency_list.push_back(near_node);
            // line(source_img, node->pos, near_node->pos, Scalar(0, 0, 255), 1);
            // imshow("PRM* for PTOPP samples", source_img);
            // waitKey(1);                
        }                    
    }
}

void PRMStarPlanner::QueryPath(Mat source_img) {
    if (GRAPH_SIZE < MAX_GRAPH_SIZE)
        ConstructRoadmap(source_img);
    
    vector<bool> visited(node_vec_.size(), false);
    UpdateNodeCost(start_node_);
    // PathNode* target_graph_node = kd_tree_.FindNearestNode(target_node_);
    PathNode* target_graph_node = FindParentByCost(node_vec_, target_node_);
    target_node_->parent = target_graph_node;
    priority_queue<PathNode*, vector<PathNode*>, PathNodeComparator> min_cost_heap;
    min_cost_heap.push(start_node_);

    plan_success_ = true;
    for (int i = 0; i < node_vec_.size(); i++) {
        while (!min_cost_heap.empty() && visited[min_cost_heap.top()->id]) {
            min_cost_heap.pop();
        }
        if (min_cost_heap.empty()) {
            // An unconnected graph is construct;
            plan_success_ = false;
            return;
        }

        PathNode* node = min_cost_heap.top();
        min_cost_heap.pop();
        visited[node->id] = true;
        if (node == target_graph_node) {
            min_cost_ = NewCost(target_graph_node, target_node_);
            return;
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

vector<float> PRMStarPlanner::QueryCostLog(vector<int> query_samples) {
    random_device rd_x, rd_y;
    mt19937 rd_engine_x(rd_x()), rd_engine_y(rd_y());
    uniform_real_distribution<> distribution_x(0, config_size_.width), distribution_y(0, config_size_.height);

    Point2f rand_pos(0, 0);
    float node_clearance = 10000;
    MAX_GRAPH_SIZE = query_samples.back();
    GRAPH_SIZE = 0;
    node_vec_.clear();
    node_vec_ = vector<PathNode*>(MAX_GRAPH_SIZE);
    node_vec_[GRAPH_SIZE++] = start_node_;
    
    while (GRAPH_SIZE < MAX_GRAPH_SIZE) {
        rand_pos.x = distribution_x(rd_x);
        rand_pos.y = distribution_y(rd_y);
        if (!PointObstacleFree(rand_pos, obstacles_))
            continue;
        if ((cost_function_type_ == 0 && plan_in_interior_) || cost_function_type_ == 1) {
            node_clearance = MinDistanceToObstaclesVec(obstacles_, rand_pos);
            if (cost_function_type_ == 0 && plan_in_interior_ && node_clearance < interior_diameter_) 
                continue; 
        }
        
        PathNode* new_node = new PathNode(rand_pos);
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
        
        float radius_alg = gamma_prm_star_ * sqrt(log(query_sample_num) / query_sample_num);
        // float radius_alg = gamma_prm_star_ * sqrt(log(MAX_GRAPH_SIZE) / MAX_GRAPH_SIZE);
        for (int i = 0; i < query_sample_num; i++) {
            PathNode* node = node_vec_[i];
            float x_min = std::max((float)0.0, node->pos.x - radius_alg), 
                x_max = std::min(node->pos.x + radius_alg, config_size_.width),
                y_min = std::max((float)0.0, node->pos.y - radius_alg), 
                y_max = std::min(node->pos.y + radius_alg, config_size_.height);  
            vector<PathNode*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max);
            for (auto near_node : near_set) {
                if (!EdgeObstacleFree(node, near_node) 
                    || cv::norm(node->pos - near_node->pos) > radius_alg
                    || near_node == node)
                    continue;
                node->adjacency_list.push_back(near_node);               
            }
        }

        vector<PathNode*> query_nodes(node_vec_.begin(), node_vec_.begin() + query_sample_num);
        vector<bool> visited(query_sample_num, false);
        UpdateNodeCost(start_node_);
        PathNode* target_graph_node = FindParentByCost(query_nodes, target_node_);
        // PathNode* target_graph_node = kd_tree_.FindNearestNode(target_node_);
        if (!target_graph_node)
            continue;
        cout << query_sample_num << "-" << target_graph_node->id << "\n";
        target_node_->parent = target_graph_node;
        priority_queue<PathNode*, vector<PathNode*>, PathNodeComparator> min_cost_heap;
        min_cost_heap.push(start_node_);
    
        for (int i = 0; i < query_sample_num; i++) {
            while (!min_cost_heap.empty() && visited[min_cost_heap.top()->id]) {
                min_cost_heap.pop();
            }
            if (min_cost_heap.empty()) {
                cout << "An unconnected graph is present\n";
                break;
            }
    
            PathNode* node = min_cost_heap.top();
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

PathNode* PRMStarPlanner::FindParentByCost(vector<PathNode*> node_vec, PathNode* new_node) {
    PathNode* parent = nullptr;
    float min_cost = FLT_MAX, min_len = FLT_MAX;
    for (PathNode* node : node_vec) {
        if (!EdgeObstacleFree(node, new_node))
            continue;

        float cost = FLT_MAX, edge_len = cv::norm(node->pos - new_node->pos);
        if (cost_function_type_ == 0)
            cost = edge_len;
        if (cost_function_type_ == 1)
            cost = -min(node->clearance, new_node->clearance);

        vector<float> passed_width_vec = GetPassedPassageWidthsDG(node, new_node, cells_info_, psg_cell_idx_map_, false);
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

void PRMStarPlanner::UpdateNodeCost(PathNode* node) {
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

float PRMStarPlanner::NewCost(PathNode* near_node, PathNode* new_node) {
    std::list<float> new_psg_list = near_node->sorted_passage_list;
    float new_len = near_node->len + cv::norm(new_node->pos - near_node->pos), 
            new_min_psg_width = near_node->min_passage_width,
            res = 0;
    vector<float> passed_width_vec;
    if (cost_function_type_ == 0)
        return new_len;
    if (cost_function_type_ == 1)
        return -min(-near_node->cost, new_node->clearance);

    if (check_all_passages_) {
        passed_width_vec = GetPassageWidthsPassed(ev_passage_pts_, near_node->pos, new_node->pos);
    }
    else { 
        passed_width_vec = GetPassedPassageWidthsDG(near_node, new_node, cells_info_, psg_cell_idx_map_, false);
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

void PRMStarPlanner::UpdateSubtree(PathNode* new_parent, PathNode* child) {
    child->parent = new_parent;
    child->len = new_parent->len + cv::norm(new_parent->pos - child->pos);

    child->min_passage_width = new_parent->min_passage_width;
    child->cur_passage_widths.clear();
    child->sorted_passage_list = new_parent->sorted_passage_list;

    vector<float> passed_width_vec;
    if (cost_function_type_ > 1) {
        if (check_all_passages_) {
            passed_width_vec = GetPassageWidthsPassed(ev_passage_pts_, new_parent->pos, child->pos);
        }
        else {
            passed_width_vec = GetPassedPassageWidthsDG(new_parent, child, cells_info_, psg_cell_idx_map_, true);
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

bool PRMStarPlanner::EdgeObstacleFree(PathNode* near_node, PathNode* new_node) {
    for (auto& obs : obstacles_)
        if (ObstacleFree(obs, near_node->pos, new_node->pos) == false)
            return false;
    return true;
}

vector<PathNode*> PRMStarPlanner::GetPath() {
    std::vector<PathNode*> res;
    if (!plan_success_) {
        std::cout << "No valid path is available.\n";
        return res;
    }
    PathNode* reverse_node = target_node_;
    while (reverse_node) {
        res.push_back(reverse_node);
        reverse_node = reverse_node->parent;
    }
    reverse(res.begin(), res.end());
    return res;   
} 

PRMStarPlanner::~PRMStarPlanner() {
    // if (start_node_) {
    //    kd_tree_.deleteTree(start_node_);
    //}
    //if (target_node_)
    //    delete target_node_;
}

PRMStarPlanner::PRMStarPlanner(const PRMStarPlanner& planner) {
      
}

PRMStarPlanner& PRMStarPlanner::operator=(const PRMStarPlanner& rhs) {
    return *this;
}
#endif