#ifndef KD_TREE3D_INCLUDED
#define KD_TREE3D_INCLUDED

#include <list>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
//using namespace std;

float SquaredNorm(const cv::Point3f& pt) {
    return pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
}

struct PathNode3d {
    int id = 0;
    int cell_id = 0;
    cv::Point3f pos;
    float cost = 0;
    float len = 0;
    float min_passage_width = 10000;
    float clearance = 10000;
    // Passage widths passed by the edge parent node---current node
    std::vector<float> cur_passage_widths;
    std::list<float> sorted_passage_list;
    // std::set<int> passage_idx_set;
    PathNode3d* parent;
    std::list<PathNode3d*> children;
    std::vector<PathNode3d*> adjacency_list;
    PathNode3d* left;
    PathNode3d* right;
    PathNode3d(): pos(cv::Point3f(0, 0, 0)), parent(nullptr), left(nullptr), right(nullptr) {}
    PathNode3d(cv::Point3f initPos): pos(initPos), parent(nullptr), left(nullptr), right(nullptr) {}
};

class PathNode3dComparator {
public: 
    bool operator() (const PathNode3d* lhs, const PathNode3d* rhs) {
        if (lhs->cost > rhs->cost)
            return true;
        else if (lhs->cost > rhs->cost - 1e-2 &&
                 lhs->len > rhs->len)
            return true;
        return false;
    }    
};

class kdTree3d{
private:
    const int kDimension_k_ = 3;
public:
    PathNode3d* kd_tree_root_;

    kdTree3d(): kd_tree_root_(nullptr) {}
    kdTree3d(PathNode3d* root_node): kd_tree_root_(root_node) {};
    ~kdTree3d();
    kdTree3d(const kdTree3d&);
    kdTree3d& operator=(const kdTree3d&);

    void AddWithRoot(PathNode3d* root, PathNode3d* new_node, int depth) {
        if (depth % kDimension_k_ == 0) {
            if (new_node->pos.x <= root->pos.x) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else 
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
        else if (depth % kDimension_k_ == 1) {
            if (new_node->pos.y <= root->pos.y) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
        else {
            if (new_node->pos.z <= root->pos.z) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
    }

    void Add(PathNode3d* new_node) {
        if (new_node == nullptr)
            return;

        if (kd_tree_root_ == nullptr)
            kd_tree_root_ = new_node;
        else
            AddWithRoot(kd_tree_root_, new_node, 0);
    }

    PathNode3d* GetCloserInTwo(PathNode3d* target, PathNode3d* candidate_1, PathNode3d* candidate_2) {
        if (candidate_1 == nullptr)
            return candidate_2;
        if (candidate_2 == nullptr)
            return candidate_1;

        if (SquaredNorm(target->pos - candidate_1->pos) <= SquaredNorm(target->pos - candidate_2->pos))
            return candidate_1;
        return candidate_2;
    }

    PathNode3d* FindNearestNodeWithRoot(PathNode3d* root, PathNode3d* target, int depth) {
        if (root == nullptr)
            return nullptr;
        
        PathNode3d *next_subtree = nullptr, *other_subtree = nullptr;
        if (depth % kDimension_k_ == 0) {
            if (target->pos.x <= root->pos.x) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }
        }
        else if (depth % kDimension_k_ == 1) {
            if (target->pos.y <= root->pos.y) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }  
        }
        else {
            if (target->pos.z <= root->pos.z) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }             
        }
        PathNode3d *temp_res = FindNearestNodeWithRoot(next_subtree, target, depth + 1),
                 *cur_best = GetCloserInTwo(target, temp_res, root);
        float cur_dist_square = SquaredNorm(target->pos - cur_best->pos), dist_to_boundary_sqr, dist_to_boundary;
        if (depth % kDimension_k_ == 0) 
            dist_to_boundary = target->pos.x - root->pos.x;
        else if (depth % kDimension_k_ == 1) 
            dist_to_boundary = target->pos.y - root->pos.y;
        else
            dist_to_boundary = target->pos.z - root->pos.z;
        dist_to_boundary_sqr = dist_to_boundary * dist_to_boundary;
        
        if (cur_dist_square >= dist_to_boundary_sqr) {
            temp_res = FindNearestNodeWithRoot(other_subtree, target, depth + 1);
            cur_best = GetCloserInTwo(target, temp_res, cur_best);
        }
        return cur_best;
    }

    PathNode3d* FindNearestNode(PathNode3d* target) {
        return FindNearestNodeWithRoot(kd_tree_root_, target, 0);
    } 
   
    PathNode3d* FindNearestNode(const cv::Point3f& target_pos) {
        PathNode3d* target_node = new PathNode3d(target_pos);
        PathNode3d* res = FindNearestNodeWithRoot(kd_tree_root_, target_node, 0);
        delete target_node;
        return res;
    }

    void RangeSearchWithRoot(PathNode3d* root, std::vector<PathNode3d*>& res_pt_vec, 
                            const float& x_min, const float& x_max, 
                            const float& y_min, const float& y_max, 
                            const float& z_min, const float& z_max,
                            int depth) {
        if (root == nullptr)
            return;

        if (root->pos.x >= x_min && root->pos.x <= x_max 
            && root->pos.y >= y_min && root->pos.y <= y_max
            && root->pos.z >= z_min && root->pos.z <= z_max)
            res_pt_vec.push_back(root);
        
        if (depth % kDimension_k_ == 0) {
            if (root->pos.x < x_min)
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            else if (root->pos.x > x_max)
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            else {
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            }   
        }
        else if (depth % kDimension_k_ == 1) {
            if (root->pos.y < y_min)
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            else if (root->pos.y > y_max)
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            else {
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            }              
        }  
        else {
            if (root->pos.z < z_min)
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            else if (root->pos.z > z_max)
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            else {
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, z_min, z_max, depth + 1);
            }                   
        }       
    }

    std::vector<PathNode3d*> RanageSearch(const float& x_min, const float& x_max, const float& y_min, const float& y_max, 
                                        const float& z_min, const float& z_max) {
        std::vector<PathNode3d*> res;
        if (x_min > x_max || y_min > y_max || z_min > z_max) {
            throw std::invalid_argument("Invalid range in range search. " + std::string(__func__));             
        }
        RangeSearchWithRoot(kd_tree_root_, res, x_min, x_max, y_min, y_max, z_min, z_max, 0);
        return res;
    }

    void deleteTree(PathNode3d* root) {
        if (root == nullptr)
            return;
        
        deleteTree(root->left);
        deleteTree(root->right);
        delete root;
    }
};

kdTree3d::~kdTree3d() {
    kdTree3d::deleteTree(kd_tree_root_);
} 

kdTree3d::kdTree3d(const kdTree3d& copied_tree) {
    kd_tree_root_ = copied_tree.kd_tree_root_;
}

kdTree3d& kdTree3d::operator=(const kdTree3d& rhs) {
    kd_tree_root_ = rhs.kd_tree_root_;
    return *this;
}
#endif