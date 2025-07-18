#ifndef DECOMPOSITION_INCLUDED
#define DECOMPOSITION_INCLUDED

#include <unordered_map>
// #include <opencv2/imgproc/imgproc.hpp>
#include "obstacles.hpp"
#include "kd_tree.hpp"

struct PolygonCell {
    vector<int> obs_indices;
    vector<Point2f> vertices;
    PolygonCell() {}
    PolygonCell(vector<int> cell_obs_indices, vector<Point2f> cell_vertices): obs_indices(cell_obs_indices), vertices(cell_vertices) {}
};

int InNegativeHalfPlane(Point2f& pt) {
    return int(pt.y < 0 || (abs(pt.y) < 1e-6 && pt.x < 0));
}

float TriplePtCross(Point2f& origin_pt, Point2f& p1, Point2f& p2) {
    return (p1 - origin_pt).cross(p2 - origin_pt);
}

string PointToString(Point2f& pt) {
    string link_symbol = "-";
    string str_x = to_string(pt.x), str_y = to_string(pt.y);
    str_x = str_x.substr(0, str_x.find(".") + 3);
    str_y = str_y.substr(0, str_y.find(".") + 3);
    return str_x + link_symbol + str_y;
}

string PassagePairToKeyString(const int obs_idx1, const int obs_idx2) {
    string link_symbol = "-";
    int min_idx = min(obs_idx1, obs_idx2),
        max_idx = max(obs_idx1, obs_idx2);
    return to_string(min_idx) + link_symbol + to_string(max_idx);
}

vector<vector<int>> DelaunayTriangulationObstables(const vector<PolygonObstacle>& obs_vec, 
                                                    bool contain_env_walls = false, 
                                                    Size2f rect_size = Size2f(10000, 10000)) {
    if (obs_vec.size() < 2) {
        string msg = "Obstacle number is less than two in " + string(__func__);
        throw std::invalid_argument(msg);        
    }   
    else if (obs_vec.size() == 2) {
        vector<vector<int>> res(2);
        res[0] = {1}; res[1] = {0};
        return res;
    }                                                     
    vector<vector<int>> adjacency_list(obs_vec.size());
    vector<set<int>> adjacency_set(obs_vec.size());
    vector<Point2f> obs_centroids_vec = GetObstaclesCentroids(obs_vec);
    // Negative coordinates are not allowed. Aassigning zero to these coordinates 
    // may cause some Delaunay edges to fail to find passages between walls and obstacles. The result 
    // could cause redundant passages whose circles intersect with walls.
    // obs_centroids_vec[0].y = 0;
    // obs_centroids_vec[2].x = 0;
    // New solution is add shift to centroids so that there are larger distances between walls and obstacles.
    Point2f shift(1000, 1000);
    for (Point2f& centroid : obs_centroids_vec)
        centroid += shift;

    unordered_map<string, int> centroid_obs_map;
    for (int i = 0; i < obs_centroids_vec.size(); i++)
        centroid_obs_map[PointToString(obs_centroids_vec[i])] = i;

    Rect2f bounding_box(0, 0, rect_size.width, rect_size.height);
    Subdiv2D subdiv(bounding_box);
    
    if (!contain_env_walls) {
        for (int i = 4; i < obs_centroids_vec.size(); i++)
            subdiv.insert(obs_centroids_vec[i]);
    }
    else {
        for (Point2f& centroid : obs_centroids_vec)
            subdiv.insert(centroid);
    }

    vector<Vec6f> triangle_list;
    subdiv.getTriangleList(triangle_list);
    for (Vec6f& triangle : triangle_list) {
        Point2f vertex_1(triangle[0], triangle[1]), 
                vertex_2(triangle[2], triangle[3]), 
                vertex_3(triangle[4], triangle[5]);
        string key_str_1 = PointToString(vertex_1),
               key_str_2 = PointToString(vertex_2),
               key_str_3 = PointToString(vertex_3);
        int obs_idx_1 = centroid_obs_map[key_str_1], 
            obs_idx_2 = centroid_obs_map[key_str_2],
            obs_idx_3 = centroid_obs_map[key_str_3];
        adjacency_set[obs_idx_1].insert(obs_idx_2);
        adjacency_set[obs_idx_1].insert(obs_idx_3);
        adjacency_set[obs_idx_2].insert(obs_idx_1);
        adjacency_set[obs_idx_2].insert(obs_idx_3);
        adjacency_set[obs_idx_3].insert(obs_idx_1);
        adjacency_set[obs_idx_3].insert(obs_idx_2);        
    }

    for (int i = 0; i < obs_vec.size(); i++) {
        adjacency_list[i] = vector<int>(adjacency_set[i].begin(), adjacency_set[i].end());
    }
    return adjacency_list;
}

Passages PassageCheckInDelaunayGraph(const vector<PolygonObstacle>& obstacles, bool contain_env_walls = true) {
    vector<vector<int>> res_psg_pair;
    vector<vector<Point2f>> res_psg_pts;
    // Computing adjacency lists needs to take into account environment walls because some passage candidates
    // between obstacles are excluded by walls. Thus walls should be involved in the adjacency list.
    vector<vector<int>> adjacency_list = DelaunayTriangulationObstables(obstacles, contain_env_walls);

    // Explicitly do not process environment walls here since passages containing walls are processed 
    // by ExtendedVisibilityCheckForWalls().
    int start_idx = 4; 
    for (int i = start_idx; i < obstacles.size(); i++) {
        // obstacle within geodesic distance (gd) two
        set<int> neighbor_obs_gd_two(adjacency_list[i].begin(), adjacency_list[i].end());
        for (int neighbor_gd_1 : adjacency_list[i])
           for (int neighbor_gd_2 : adjacency_list[neighbor_gd_1]) 
                neighbor_obs_gd_two.insert(neighbor_gd_2);

        for (int j : neighbor_obs_gd_two) {
            if (j <= i)
                continue;

            vector<vector<Point2f>> psg_key_pts = SVIntersection(obstacles[i], obstacles[j]);
            vector<Point2f> psg_segment_pts = psg_key_pts.back();
            float psg_length = cv::norm(psg_segment_pts[0] - psg_segment_pts[1]);
            // obstacle within geodesic distance (gd) two of two obstacles
            set<int> obs_gd_two = neighbor_obs_gd_two;
            for (int k : adjacency_list[j]) {
                obs_gd_two.insert(k);
                for (int l : adjacency_list[k])
                    obs_gd_two.insert(l);
            }

            bool is_psg_valid = true;
            for (int k : obs_gd_two) {
                if (k == i || k == j)
                    continue;
                
                // Assumption: if an obstacle collides with the passage region,   
                // it must collide with  one of the passage region boundaries.
                if (!ObstacleFree(obstacles[k], psg_key_pts[0][0], psg_key_pts[0][1])
                    || !ObstacleFree(obstacles[k], psg_key_pts[1][0], psg_key_pts[1][1])) {
                    is_psg_valid = false;
                    break;
                }
                Point2f psg_center = (psg_segment_pts[0] + psg_segment_pts[1]) / 2;
                float obs_psg_center_dist = MinDistanceToObstacle(obstacles[k], psg_center);
                if (obs_psg_center_dist <= psg_length / 2) {
                    is_psg_valid = false;                    
                    break;
                }
            }
            if (is_psg_valid) {
                res_psg_pair.push_back({i, j});
                res_psg_pts.push_back(psg_segment_pts);
            }
        }
    }
    return Passages(res_psg_pair, res_psg_pts);
}

Passages PassageCheckInDelaunayGraphWithGeodesicDistance(const vector<PolygonObstacle>& obstacles, 
                                                        int geodesic_distance = 1,
                                                        bool contain_env_walls = true) {
    vector<vector<int>> res_psg_pair;
    vector<vector<Point2f>> res_psg_pts;
    vector<vector<int>> adjacency_list = DelaunayTriangulationObstables(obstacles, contain_env_walls);

    int start_idx = 4; 
    for (int i = start_idx; i < obstacles.size(); i++) {
        // obstacle within geodesic distance (gd) two
        set<int> neighbor_obs_gd_two(adjacency_list[i].begin(), adjacency_list[i].end());

        for (int j : neighbor_obs_gd_two) {
            if (j <= i)
                continue;

            vector<vector<Point2f>> psg_key_pts = SVIntersection(obstacles[i], obstacles[j]);
            vector<Point2f> psg_segment_pts = psg_key_pts.back();
            float psg_length = cv::norm(psg_segment_pts[0] - psg_segment_pts[1]);
            // obstacle within geodesic distance (gd) two of two obstacles
            set<int> obs_gd_two = neighbor_obs_gd_two;
            for (int k : adjacency_list[j]) {
                obs_gd_two.insert(k);
            }

            bool is_psg_valid = true;
            for (int k : obs_gd_two) {
                if (k == i || k == j)
                    continue;
                
                if (!ObstacleFree(obstacles[k], psg_key_pts[0][0], psg_key_pts[0][1])
                    || !ObstacleFree(obstacles[k], psg_key_pts[1][0], psg_key_pts[1][1])) {
                    is_psg_valid = false;
                    break;
                }
                Point2f psg_center = (psg_segment_pts[0] + psg_segment_pts[1]) / 2;
                float obs_psg_center_dist = MinDistanceToObstacle(obstacles[k], psg_center);
                if (obs_psg_center_dist <= psg_length / 2) {
                    is_psg_valid = false;                    
                    break;
                }
            }
            if (is_psg_valid) {
                res_psg_pair.push_back({i, j});
                res_psg_pts.push_back(psg_segment_pts);
            }
        }
    }
    return Passages(res_psg_pair, res_psg_pts);
}

Passages PassageCheckDelaunayGraphWithWalls(const vector<PolygonObstacle>& obstacles) {
    Passages res_env_walls = ExtendedVisibilityCheckForWalls(obstacles);
    Passages res_obs = PassageCheckInDelaunayGraph(obstacles, true);
    res_env_walls.pairs.insert(res_env_walls.pairs.end(), res_obs.pairs.begin(), res_obs.pairs.end());
    res_env_walls.pts.insert(res_env_walls.pts.end(), res_obs.pts.begin(), res_obs.pts.end());
    return res_env_walls;
}

vector<vector<int>> FindPlannarFaces(const vector<PolygonObstacle>& obstacles, const vector<vector<int>>& passage_pairs) {
    int obs_num = obstacles.size();
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);
    // Expand centroids of environment walls so that their relative  
    // directions w.r.t. one another are correctly computed.
    obs_centroids[0].y -= 10000;
    obs_centroids[1].y += 10000;
    obs_centroids[2].x -= 10000;
    obs_centroids[3].x += 10000;

    vector<vector<int>> adjacency_list(obs_num);
    for (const vector<int>& passage_pair : passage_pairs) {
        adjacency_list[passage_pair[0]].push_back(passage_pair[1]);
        adjacency_list[passage_pair[1]].push_back(passage_pair[0]);
    }
    
    vector<vector<char>> used(obs_num);
    for (int i = 0; i < obs_num; i++) {
        used[i].resize(adjacency_list[i].size());
        used[i].assign(adjacency_list[i].size(), 0);
        // Sort points by axis angles. Two cases for transitivity: 1) on the same half plane, 
        // 2) on different half planes. Solely using cross product is insufficient in comparer.
        auto compare = [&](int l, int r) {
            Point2f pl = obs_centroids[l] - obs_centroids[i];
            Point2f pr = obs_centroids[r] - obs_centroids[i];
            if (InNegativeHalfPlane(pl) != InNegativeHalfPlane(pr))
                return InNegativeHalfPlane(pl) < InNegativeHalfPlane(pr);
            return pl.cross(pr) > 0;
        };
        std::sort(adjacency_list[i].begin(), adjacency_list[i].end(), compare);
    }

    vector<vector<int>> faces;
    for (int i = 0; i < obs_num; i++) {
        for (int edge_id = 0; edge_id < adjacency_list[i].size(); edge_id++) {
            if (used[i][edge_id]) {
                continue;
            }
            vector<int> face;
            int v = i;
            int e = edge_id;
            while (!used[v][e]) {
                used[v][e] = true;
                face.push_back(v);
                int u = adjacency_list[v][e];
                int e_1 = std::lower_bound(adjacency_list[u].begin(), adjacency_list[u].end(), v, [&](int l, int r) {
                    Point2f pl = obs_centroids[l] - obs_centroids[u];
                    Point2f pr = obs_centroids[r] - obs_centroids[u];
                    if (InNegativeHalfPlane(pl) != InNegativeHalfPlane(pr))
                        return InNegativeHalfPlane(pl) < InNegativeHalfPlane(pr);
                    return pl.cross(pr) > 0;
                }) - adjacency_list[u].begin() + 1;
                if (e_1 == adjacency_list[u].size()) {
                    e_1 = 0;
                }
                v = u;
                e = e_1;
            }
            
            // Obstacles in cells are in counterclockwise order except the outerest cell.
            // If a clockwise order of obstacles is preferred, reverse face.
            // std::reverse(face.begin(), face.end());
            int sign = 0;
            for (int j = 0; j < face.size(); j++) {
                int j_1 = (j + 1) % face.size();
                int j_2 = (j + 2) % face.size();
                float val = TriplePtCross(obs_centroids[face[j]], obs_centroids[face[j_1]], obs_centroids[face[j_2]]);
                if (val > 0) {
                    sign = 1;
                    break;
                } else if (val < 0) {
                    sign = -1;
                    break;
                }
            }
            if (sign <= 0) {
                faces.insert(faces.begin(), face);
            } else {
                faces.emplace_back(face);
            }
        }
    }
    return faces;
}

vector<vector<int>> ReportGabrielCells(const vector<PolygonObstacle>& obstacles, const vector<vector<int>>& passage_pairs, bool contain_env_walls) {
    vector<vector<int>> augmented_psg_pairs = passage_pairs;
    // Add pseudo passage between four environment walls.
    if(contain_env_walls) {
        augmented_psg_pairs.push_back({0, 2});
        augmented_psg_pairs.push_back({0, 3});
        augmented_psg_pairs.push_back({1, 2});
        augmented_psg_pairs.push_back({1, 3});
    }
    return FindPlannarFaces(obstacles, augmented_psg_pairs);
}

vector<PolygonCell> GetGabrielCellsInfo(const vector<vector<int>>& cells, 
                                        const Passages& psg_res) {
    vector<PolygonCell> res;
    // Add virtual passage between four environment walls.
    vector<vector<int>> augmented_psg_pairs = psg_res.pairs;
    augmented_psg_pairs.push_back({0, 2});
    augmented_psg_pairs.push_back({0, 3});
    augmented_psg_pairs.push_back({1, 2});
    augmented_psg_pairs.push_back({1, 3});
    // Set invalid passage points such that no intersection with path segments is possible.
    vector<vector<Point2f>> augmented_psg_pts = psg_res.pts;
    augmented_psg_pts.push_back({Point2f(0, -1), Point2f(-1, 0)});
    augmented_psg_pts.push_back({Point2f(10000, -1), Point2f(10001, 0)});
    augmented_psg_pts.push_back({Point2f(0, 10001), Point2f(-1, 10000)});
    augmented_psg_pts.push_back({Point2f(10000, 10001), Point2f(10001, 10000)});   

    unordered_map<string, vector<Point2f>> psg_to_pt_map;
    int psg_num = augmented_psg_pairs.size();
    for (int i = 0; i < psg_num; i++) {
        string key_str = PassagePairToKeyString(augmented_psg_pairs[i][0], augmented_psg_pairs[i][1]);
        psg_to_pt_map[key_str] = augmented_psg_pts[i];
    }

    int cell_num = cells.size();
    for (int i = 0; i < cell_num; i++) {
        // Skip the outer cell comprised of environment walls 0, 1, 2, 3.
        if (cells[i].size() == 4 && accumulate(cells[i].begin(), cells[i].end(), 0) == 6)
            continue;
        
        int vertex_num = cells[i].size();
        vector<Point2f> cell_side_pts;
        for (int j = 0; j < vertex_num; j++) {
            int start_idx = cells[i][j], end_idx = cells[i][(j + 1) % vertex_num];
            string key_str = PassagePairToKeyString(start_idx, end_idx);
            
            if (psg_to_pt_map.count(key_str) == 0) 
                throw std::runtime_error("Error: Passage as a cell side does not exist in detection results in " + string(__func__));

            // Passage segment points are stored with obstacle indices ascending in 
            // passage detection results. In cells, however, obstacles are not ordered. 
            if (start_idx < end_idx) {
                cell_side_pts.push_back(psg_to_pt_map[key_str][0]);
                cell_side_pts.push_back(psg_to_pt_map[key_str][1]);
            }
            else {
                cell_side_pts.push_back(psg_to_pt_map[key_str][1]);
                cell_side_pts.push_back(psg_to_pt_map[key_str][0]);                
            }
        }
        res.push_back(PolygonCell(cells[i], cell_side_pts));
    }
    return res;
}

unordered_map<string, vector<int>> GetPassageCellMap(const vector<PolygonCell>& cell_info) {
    unordered_map<string, vector<int>> psg_cell_idx_map; 
    for (int i = 0; i < cell_info.size(); i++) {
        int vertex_num = cell_info[i].obs_indices.size();
        for (int j = 0; j < vertex_num; j++) {
            string key_str = PassagePairToKeyString(cell_info[i].obs_indices[j], cell_info[i].obs_indices[(j + 1) % vertex_num]);
            psg_cell_idx_map[key_str].push_back(i);
        }
    }
    return psg_cell_idx_map;
}

int LocatePtInCells(const Point2f pt, const vector<PolygonCell>& cell_info) {
    for (int i = 0; i < cell_info.size(); i++) {
        if (InsidePolygon(pt, cell_info[i].vertices))
            return i;
    }
    return -1;
}

int LocatePtInGivenCells(const Point2f pt, const vector<PolygonCell>& cell_info, const vector<int>& given_indices) {
    for (int i : given_indices) {
        if (InsidePolygon(pt, cell_info[i].vertices))
            return i;
    }
    return -1;
}


vector<float> GetPassedPassageWidthsDG(PathNode* start_node, PathNode* end_node,
                                       const vector<PolygonCell>& cells_info, 
                                       unordered_map<string, vector<int>>& psg_cell_idx_map,
                                       bool update_end_node_cell_idx = false) {
    vector<float> res;
    set<string> passed_psg_pairs;
    int start_cell_idx = start_node->cell_id;
    Point2f start_pt = start_node->pos, end_pt = end_node->pos;

    bool end_pt_located = false;
    while (!end_pt_located) {
        end_pt_located = true;
        const PolygonCell* cell = &cells_info[start_cell_idx];
        int side_num = cell->obs_indices.size();
        for (int i = 0; i < side_num; i++) {
            Point2f vertex_1 = cell->vertices[2 * i],
                    vertex_2 = cell->vertices[2 * i + 1];
            if (SegmentIntersection(vertex_1, vertex_2, start_pt, end_pt)) {
                string key_str = PassagePairToKeyString(cell->obs_indices[i], cell->obs_indices[(i + 1) % side_num]);
                if (passed_psg_pairs.count(key_str) > 0)
                    continue;

                res.push_back(cv::norm(vertex_1 - vertex_2));
                passed_psg_pairs.insert(key_str);
                for (int cell_idx : psg_cell_idx_map[key_str]) {
                    if (cell_idx != start_cell_idx) {
                        start_cell_idx = cell_idx;
                        end_pt_located = false;
                        break;
                    }
                }
                break;
            }
        }
    }
    if (update_end_node_cell_idx)
        end_node->cell_id = start_cell_idx;
    return res;
}

#endif