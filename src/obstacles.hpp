#ifndef OBSTACLES_INCLUDED
#define OBSTACLES_INCLUDED

#include <math.h>
#include <iostream>
#include <vector>
#include <set>
#include <list>
#include <algorithm>
#include <random>
#include <ctime>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

using namespace cv;
using namespace std;

struct PolygonObstacle {
    vector<Point2f> vertices;
    PolygonObstacle() {}
    PolygonObstacle(const vector<Point2f>& polygon_vertices): vertices(polygon_vertices) {}
    PolygonObstacle(const vector<Point>& polygon_vertices) {
        vertices = vector<Point2f>(polygon_vertices.size(), Point2f(0, 0));
        for (int i = 0; i < polygon_vertices.size(); i++) {
            vertices[i].x = (float) polygon_vertices[i].x;
            vertices[i].y = (float) polygon_vertices[i].y;
        }
    }                                                              
};

struct Passage {
    vector<int> pair;
    vector<Point2f> pts;
    Passage() {}
    Passage(const vector<int>& input_pair, const vector<Point2f>& input_pts): pair(input_pair), pts(input_pts) {}
};

struct Passages {
    vector<vector<int>> pairs;
    vector<vector<Point2f>> pts;
    Passages() {}
    Passages(const vector<vector<int>>& input_pairs, const vector<vector<Point2f>>& input_pts): pairs(input_pairs), pts(input_pts) {}
};

float NormSqr(const Point2f& pt) {
    return pt.x * pt.x + pt.y * pt.y;
}

float CrossProductVal(const Point2f& pt_1, const Point2f& pt_2) {
    return pt_1.x * pt_2.y - pt_1.y * pt_2.x;
}

void InsertIntoSortedList(list<float>& input_list, const float val) {
    if (input_list.size() == 0) {
        input_list.push_back(val);
        return;
    }

    list<float>::iterator it = input_list.begin();
    while (it != input_list.end() && *it < val)
        it++;
    // if (abs(*it - val) < 1e-3)
    //    return;
    // else
    input_list.insert(it, val);
}

void InsertIntoSortedList(list<float>& input_list, const vector<float> val_vec) {
    for (const float& val : val_vec)
        InsertIntoSortedList(input_list, val);
}

vector<Point2f> GetObstaclesCentroids(const vector<PolygonObstacle>& obstacles) {
    vector<Point2f> obs_centroids(obstacles.size());
    for (int i = 0; i < obstacles.size(); i++) {
        obs_centroids[i] = Point2f(0, 0);
        for (const Point2f& vertex : obstacles[i].vertices) 
            obs_centroids[i] += vertex;
        obs_centroids[i] /= (float) obstacles[i].vertices.size();
    } 
    return obs_centroids;
}

void DrawDashedLine(Mat img, const Point2f& initial_pt, const Point2f& end_pt, Scalar color = Scalar(0, 0, 0), int thickness = 2, float dash_len = 5) {
    float line_len = cv::norm(end_pt - initial_pt);
    Point2f line_direction = (end_pt -initial_pt) / line_len;
    int dash_num = line_len / dash_len;
    if (line_len < dash_len)
        line(img, initial_pt, end_pt, color, thickness);
    else if (line_len < 2 * dash_len) 
        line(img, initial_pt, initial_pt + dash_len * line_direction, color, thickness);
    for (int i = 0; i <= dash_num; i += 2) 
        if (i == dash_num)
            line(img, initial_pt + i * dash_len * line_direction, end_pt, color, thickness);
        else
            line(img, initial_pt + i * dash_len * line_direction, initial_pt + (i + 1) * dash_len * line_direction, color, thickness);
}

void DrawObstacles(Mat back_img, const vector<PolygonObstacle>& obs_vec, bool put_obs_idx = true) {
    int obs_num = obs_vec.size();
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obs_vec);

    for (int i = 0; i < obs_num; i++) {
        PolygonObstacle obs = obs_vec[i];
        int cur_vertex_num = obs.vertices.size();
        for (int j = 0; j < cur_vertex_num; j++) {
            line(back_img, obs.vertices[j], obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
        }
        if (put_obs_idx)
            putText(back_img, std::to_string(i), obs_centroids[i] - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
    }      
}

int Orientation(const Point2f& p1, const Point2f& p2, const Point2f& p3) {
    float threshold = 1e-6;
    // Normalization is important to mitigate the threshold error brought by value magnitude.
    Point2f vec1 = (p2 - p1) / cv::norm(p2 - p1), vec2 = (p3 - p1) / cv::norm(p3 - p1);
    float cross_product = CrossProductVal(vec1, vec2);
    // From segment p1-p2 to p1-p3, counterclockwise direction, CCW
    if (cross_product > threshold)
        return 1;   
    // clockwise direction
    else if (cross_product < -threshold) 
        return 2;    
    else
        return 0;
}

/// check if point q is on the segment p1-p2 when the three points are colinear
bool OnSegment(const Point2f& p1, const Point2f& p2, const Point2f& q) {  
    Point2f vec_1 = p1 - q, vec_2 = p2 - q;
    float inner_product = vec_1.x * vec_2.x + vec_1.y * vec_2.y;
    return inner_product <= 0;
}

/// segments: p1-p2, q1-q2
bool SegmentIntersection(const Point2f& p1, const Point2f& p2, const Point2f& q1, const Point2f& q2) {
    int ori1 = Orientation(p1, p2, q1),  
        ori2 = Orientation(p1, p2, q2),
        ori3 = Orientation(q1, q2, p1),
        ori4 = Orientation(q1, q2, p2);
    // cout << ori1 << "-" << ori2 << "-" << ori3 << "-" << ori4 << "\n";
    
    if (ori1 != ori2 && ori3 != ori4 && ori1 * ori2 != 0 && ori3 * ori4 != 0)
        return true;
    else if (ori1 == 0 && OnSegment(p1, p2, q1))
        return true;
    else if (ori2 == 0 && OnSegment(p1, p2, q2))
        return true;
    else if (ori3 == 0 && OnSegment(q1, q2, p1))
        return true;
    else if (ori4 == 0 && OnSegment(q1, q2, p2))
        return true;
    return false;
}

bool InsidePolygon(const Point2f& pt, const vector<Point2f>& vertices) {
    if (vertices.size() < 3)
        throw std::invalid_argument("Error: A polygon with a vertex number less than three in " + string(__func__));

    float range_x_min = FLT_MAX, range_x_max = -FLT_MAX,
          range_y_min = FLT_MAX, range_y_max = -FLT_MAX;
    for (const Point2f& vertex : vertices) {
        range_x_min = min(range_x_min, vertex.x);
        range_x_max = max(range_x_max, vertex.x);
        range_y_min = min(range_y_min, vertex.y);
        range_y_max = max(range_y_max, vertex.y);
    }
    if (pt.x < range_x_min || pt.x > range_x_max || pt.y < range_y_min || pt.y > range_y_max)
        return false;
    
    // Do not simply use vertically or horizontal far points. It will cause the singlular case
    // wheret the test line intersect with a polygon vertex easily in PTOPP. In this case, the 
    // intersection time become 2.
    Point2f far_pt = pt + Point2f(1e5, 1e5);  
    int vertex_num = vertices.size(), intersection_cnt = 0;
    for (int i = 0; i < vertices.size(); i++) {
        if (SegmentIntersection(pt, far_pt, vertices[i], vertices[(i + 1) % vertex_num])) 
            intersection_cnt++;
    }
    // cout << pt << " intersection times " << intersection_cnt << "\n";
    return intersection_cnt % 2 == 1;
}

bool PointObstacleFree(const Point2f& pt, const vector<PolygonObstacle>& obs_vec) {
    for (const PolygonObstacle& obs : obs_vec) {
        if (InsidePolygon(pt, obs.vertices))
            return false;
    }
    return true;
}

/// Return the point on segment p1-p2 closest to test_pt.
Point2f ClosestPtOnSegmentToPt(const Point2f& p1, const Point2f& p2, const Point2f& test_pt) {
    Point2f p_1_to_test_pt_vec = test_pt - p1,
            segment_vec = p2 - p1,
            res_pt;
    float inner_product = p_1_to_test_pt_vec.x * segment_vec.x + p_1_to_test_pt_vec.y * segment_vec.y;
    float segment_len_square = segment_vec.x * segment_vec.x + segment_vec.y * segment_vec.y;
    float proj_segment_len_ratio = inner_product / segment_len_square;
    if (proj_segment_len_ratio < 0)
        res_pt = p1;
    else if (proj_segment_len_ratio > 1)
        res_pt = p2;
    else 
        res_pt = p1 + proj_segment_len_ratio * segment_vec;
    return res_pt;
}

 /// Segment p1-p2, q1-q2 intersect, return the intersection point. Otherwise will return intersection point of two lines.
Point2f GetSegmentsIntersectionPt(const Point2f& p1, const Point2f& p2, const Point2f& q1, const Point2f& q2) {
    if (abs(p1.x - p2.x) < 1e-2 && abs(q1.x - q2.x) > 1e-2)
        return Point2f(p1.x, q1.y + (q1.y - q2.y) / (q1.x - q2.x) * (p1.x - q1.x));
    else if (abs(p1.x - p2.x) > 1e-2 && abs(q1.x - q2.x) < 1e-2)
        return Point2f(q1.x, p1.y + (p1.y - p2.y) / (p1.x - p2.x) * (q1.x - p1.x));
    else if (abs(p1.x - p2.x) < 1e-2 && abs(q1.x - q2.x) < 1e-2 && abs(p1.x - q1.x) < 1e-2) {
        float p_min_y = min(p1.y, p2.y), p_max_y = max(p1.y, p2.y),
              q_min_y = min(q1.y, q2.y), q_max_y = max(q1.y, q2.y);
        if (q1.y >= p_min_y && q1.y <= p_max_y)
            return q1;
        else if (q2.y >= p_min_y && q2.y <= p_max_y)
            return q2;
        else if (p1.y >= q_min_y && p1.y <= q_max_y)
            return p1;
        else if (p2.y >= q_min_y && p2.y <= q_max_y)
            return p2;
        return Point2f(p1.x, 0);        
    }
    else if (abs(p1.x - p2.x) < 1e-2 && abs(q1.x - q2.x) < 1e-2) {
        string msg = "Receive parallel non-intersecting vertical lines as arguments in " + string(__func__);
        throw std::invalid_argument(msg);
    }

    float kp = (p2.y - p1.y) / (p2.x - p1.x),
          kq = (q2.y - q1.y) / (q2.x - q1.x),
          x_p1 = p1.x, y_p1 = p1.y,
          x_q1 = q1.x, y_q1 = q1.y;

    // Due to projections on regular shapes, e.g., rectangles, parallel and overlapping 
    // segments are often input. Careful processing is needed to make procedures sound.
    if (abs(kp - kq) < 1e-4) {
        Point2f p_vec = (p2 - p1) / cv::norm(p2 - p1), pq_vec = q1 - p1;
        float inner_product = p_vec.x * pq_vec.x + p_vec.y * pq_vec.y,
              line_dist = cv::norm(pq_vec - inner_product * p_vec);

        if (OnSegment(p1, p2, q1) && !OnSegment(p1, p2, q2))
            return q1;
        else if (!OnSegment(p1, p2, q1) && OnSegment(p1, p2, q2))
            return q2;
        else if (OnSegment(p1, p2, q1) && OnSegment(p1, p2, q2)) {
            // Return the point closer to midpoint of p1, p2 so that invoking 
            // in the shadow volume intersection procedure can work well.
            Point2f ref_pt = (p1 + p2) / 2;
            return NormSqr(ref_pt - q1) < NormSqr(ref_pt - q2) ? q1 : q2;
        }  
        else if (OnSegment(q1, q2, p1) && !OnSegment(q1, q2, p2))
            return p1;
        else if (!OnSegment(q1, q2, p1) && OnSegment(q1, q2, p2))
            return p2;
        else if (OnSegment(q1, q2, p1) && OnSegment(q1, q2, p2)) {
            Point2f ref_pt = (p1 + p2) / 2;
            return NormSqr(ref_pt - p1) < NormSqr(ref_pt - p2) ? p1 : p2;
        }     
        else if (line_dist > 1e-2) { 
            // It turns out that using line distance or cross product to judge if two segments are parallel and  
            // not coincident is not accurate due to rounding error. The computed distance of two coincident segments 
            // can be 0.0x or even 0.x, much larger than zero.
            throw std::invalid_argument("Receive parallel but non-intersecting segments as arguments in " + string(__func__));  
         }     
        else 
            return p1;
    }
    float x = (y_p1 - y_q1 - kp * x_p1 + kq * x_q1) / (kq - kp),
          y = y_p1 + kp * (x - x_p1);
    return Point2f(x, y);
}

float MinDistanceToObstacle(const PolygonObstacle& obs, Point2f testPt) {
    float res = FLT_MAX;
    int obs_vertices_num = obs.vertices.size();
    for (int i = 0; i < obs_vertices_num; i++) {
        Point2f closest_pt = ClosestPtOnSegmentToPt(obs.vertices[i], obs.vertices[(i + 1) % obs_vertices_num], testPt);
        float distance = cv::norm(testPt - closest_pt);
        res = min(res, distance);
    }
    return res;
}

float MinDistanceToObstaclesVec(const vector<PolygonObstacle>& obstacles, Point2f testPt) {
    float res = FLT_MAX, distance_to_obs;
    for (const PolygonObstacle& obs : obstacles) {
        distance_to_obs = MinDistanceToObstacle(obs, testPt);
        if (distance_to_obs < res)
            res = distance_to_obs;
    }
    return res;
}

bool ObstacleFree(const PolygonObstacle& obs, Point2f p1, Point2f p2) {
    if (obs.vertices.size() <= 1)
        return true;
    
    int vertex_num = obs.vertices.size();
    for (int i = 0; i < vertex_num; i++) {
        if (SegmentIntersection(obs.vertices[i], obs.vertices[(i + 1) % vertex_num], p1, p2)) 
            return false;
    }
    return true;
}

bool ObstacleFreeVec(vector<PolygonObstacle>& obs, Point2f p1, Point2f p2) {
    for (PolygonObstacle& obstacle : obs)
        if (!ObstacleFree(obstacle, p1, p2))
            return false;
    return true;
}

bool ObstaclesIntersect(const PolygonObstacle& obs1, const PolygonObstacle& obs2) {
    int vertex_num_1 = obs1.vertices.size();
    for (int i = 0; i < vertex_num_1; i++) 
        if (!ObstacleFree(obs2, obs1.vertices[i], obs1.vertices[(i + 1) % vertex_num_1]))
            return true;    
    // If obstacle sides do not intersect, check if one is inside the other one.
    if (InsidePolygon(obs1.vertices[0], obs2.vertices) || InsidePolygon(obs2.vertices[0], obs1.vertices))
        return true;
    return false;
}

vector<Point2f> GetPassageSegmentPts(const PolygonObstacle& obs1, const PolygonObstacle& obs2) {
    int vertices_num_1 = obs1.vertices.size(), vertices_num_2 = obs2.vertices.size();
    float min_dist_sqr = FLT_MAX;
    Point2f res_pt_1, res_pt_2;
    for (int i = 0; i < vertices_num_1; i++)
        for (int j = 0; j < vertices_num_2; j++) {
            Point2f pt_2 = ClosestPtOnSegmentToPt(obs2.vertices[j], obs2.vertices[(j + 1) % vertices_num_2], obs1.vertices[i]);
            float dist_sqr = NormSqr(obs1.vertices[i] - pt_2); 
            if (dist_sqr < min_dist_sqr) {
                min_dist_sqr = dist_sqr;
                res_pt_1 = obs1.vertices[i];
                res_pt_2 = pt_2;
            }
        }
    for (int i = 0; i < vertices_num_2; i++)
        for (int j = 0; j < vertices_num_1; j++) {
            Point2f pt_1 = ClosestPtOnSegmentToPt(obs1.vertices[j], obs1.vertices[(j + 1) % vertices_num_1], obs2.vertices[i]);
             float dist_sqr = NormSqr(obs2.vertices[i] - pt_1); 
            if (dist_sqr < min_dist_sqr) {
                min_dist_sqr = dist_sqr;
                res_pt_1 = pt_1;
                res_pt_2 = obs2.vertices[i];
            }           
        }
    return {res_pt_1, res_pt_2};
}

float DistanceBetweenObstacles(const PolygonObstacle& obs1, const PolygonObstacle& obs2) {
    vector<Point2f> psg_pts = GetPassageSegmentPts(obs1, obs2);
    return cv::norm(psg_pts[0] - psg_pts[1]);
}

/// p1-p2 may intersect with two sides of a convex obstacle, return the intersection point closer to testPt.
Point2f GetClosestIntersectionPt(const PolygonObstacle& obs, Point2f p1, Point2f p2, Point2f testPt) {
    Point2f res = Point2f(0, 0);
    if (obs.vertices.size() <= 1) {
        throw std::invalid_argument("Obstacle has vertices less than one in " + string(__func__));
    }

    int vertex_num = obs.vertices.size();
    float min_distance_sqr = FLT_MAX;
    for (int i = 0; i < obs.vertices.size(); i++) {
        Point2f side_direction = obs.vertices[(i + 1) % vertex_num] - obs.vertices[i];
        side_direction /= cv::norm(side_direction);
        // Prolonging obstacle sides a little helpes to make robust computations in corner cases.
        Point2f prolong_vertex_1 = obs.vertices[i] - 0.5 * side_direction, 
                prolong_vertex_2 = obs.vertices[(i + 1) % vertex_num] + 0.5 * side_direction;
        if (SegmentIntersection(p1, p2, prolong_vertex_1, prolong_vertex_2)) {
            Point2f intersection_pt = GetSegmentsIntersectionPt(p1, p2, prolong_vertex_1, prolong_vertex_2);
            if (NormSqr(intersection_pt - testPt) < min_distance_sqr) {
                res = intersection_pt;
                min_distance_sqr = NormSqr(intersection_pt - testPt);
            } 
        }
    }
    return res;
}

/// Shadow volume intersection detection
vector<vector<Point2f>> SVIntersection(const PolygonObstacle& obs1, const PolygonObstacle& obs2) {
    vector<Point2f> psg_seg_pts = GetPassageSegmentPts(obs1, obs2);
    Point2f psg_seg_vec = (psg_seg_pts[0] - psg_seg_pts[1]) / cv::norm(psg_seg_pts[0] - psg_seg_pts[1]),
            psg_seg_normal = Point2f(-psg_seg_vec.y, psg_seg_vec.x),
            psg_center = (psg_seg_pts[0] + psg_seg_pts[1]) / 2,
            max_proj_pt_1, max_proj_pt_2, min_proj_pt_1, min_proj_pt_2;
    
    float max_proj_1 = -FLT_MAX, min_proj_1 = FLT_MAX, max_dist_to_psg_center = 0;
    for (const Point2f& vertex: obs1.vertices) {
        Point2f vec = vertex - psg_center;
        max_dist_to_psg_center = max(max_dist_to_psg_center, (float)cv::norm(vec));
        float proj = vec.x * psg_seg_normal.x + vec.y * psg_seg_normal.y;
        if (proj > max_proj_1) {
            max_proj_1 = proj;
            max_proj_pt_1 = psg_center + proj * psg_seg_normal;
        }
        if (proj < min_proj_1) {
            min_proj_1 = proj;
            min_proj_pt_1 = psg_center + proj * psg_seg_normal;
        }
    }
    
    float max_proj_2 = -FLT_MAX, min_proj_2 = FLT_MAX;
    for (const Point2f& vertex: obs2.vertices) {
        Point2f vec = vertex - psg_center;
        max_dist_to_psg_center = max(max_dist_to_psg_center, (float)cv::norm(vec));
        float proj = vec.x * psg_seg_normal.x + vec.y * psg_seg_normal.y;
        if (proj > max_proj_2) {
            max_proj_2 = proj;
            max_proj_pt_2 = psg_center + proj * psg_seg_normal;
        }
        if (proj < min_proj_2) {
            min_proj_2 = proj;
            min_proj_pt_2 = psg_center + proj * psg_seg_normal;
        }
    }
    
    Point2f SVI_max_ref_pt = (max_proj_1 < max_proj_2) ? max_proj_pt_1 : max_proj_pt_2,
            SVI_min_ref_pt = (min_proj_1 > min_proj_2) ? min_proj_pt_1 : min_proj_pt_2;

    Point2f SVI_max_side_pt_1 = SVI_max_ref_pt + psg_seg_vec * max_dist_to_psg_center,
            SVI_max_side_pt_2 = SVI_max_ref_pt - psg_seg_vec * max_dist_to_psg_center,
            SVI_min_side_pt_1 = SVI_min_ref_pt + psg_seg_vec * max_dist_to_psg_center,
            SVI_min_side_pt_2 = SVI_min_ref_pt - psg_seg_vec * max_dist_to_psg_center;

    SVI_max_side_pt_1 = GetClosestIntersectionPt(obs1, SVI_max_side_pt_1, SVI_max_side_pt_2, SVI_max_ref_pt),
    SVI_max_side_pt_2 = GetClosestIntersectionPt(obs2, SVI_max_side_pt_1, SVI_max_side_pt_2, SVI_max_ref_pt),
    SVI_min_side_pt_1 = GetClosestIntersectionPt(obs1, SVI_min_side_pt_1, SVI_min_side_pt_2, SVI_min_ref_pt),
    SVI_min_side_pt_2 = GetClosestIntersectionPt(obs2, SVI_min_side_pt_1, SVI_min_side_pt_2, SVI_min_ref_pt);
    
    vector<vector<Point2f>> res({{SVI_max_side_pt_1, SVI_max_side_pt_2}, 
                                 {SVI_min_side_pt_1, SVI_min_side_pt_2}, 
                                 psg_seg_pts});  
    return res;
}

/// Return the min width of passages the segment pt1-pt2 passes. Return -1 if no passage is passed
float GetMinPassageWidthPassed(const vector<vector<Point2f>>& passage_pts, Point2f pt1, Point2f pt2) {
    float res = FLT_MAX;

    for (int i = 0; i < passage_pts.size(); i++) {
        if (SegmentIntersection(passage_pts[i][0], passage_pts[i][1], pt1, pt2))
            res = min(res, (float)cv::norm(passage_pts[i][0] - passage_pts[i][1]));
    }
    
    if (res > FLT_MAX / 2) 
        return -1.0;
    return res;
}

vector<float> GetPassageWidthsPassed(const vector<vector<Point2f>>& passage_pts, Point2f pt1, Point2f pt2) {
    vector<float> res;
    for (int i = 0; i < passage_pts.size(); i++) {
        if (SegmentIntersection(passage_pts[i][0], passage_pts[i][1], pt1, pt2))
            res.push_back(cv::norm(passage_pts[i][0] - passage_pts[i][1]));
    }
    return res;
}

int GetMinPassageIdxPassed(const vector<vector<Point2f>>& passage_pts, Point2f pt1, Point2f pt2) {
    int res_idx = -1;
    float min_width = FLT_MAX;

    for (int i = 0; i < passage_pts.size(); i++) {
        if (SegmentIntersection(passage_pts[i][0], passage_pts[i][1], pt1, pt2)) {
            float width = cv::norm(passage_pts[i][0] - passage_pts[i][1]);
            if (width < min_width) {
                min_width = width;
                res_idx = i;
            }
        }
    }
    return res_idx;
}

vector<PolygonObstacle> GenerateRandomObstacles(int obstacle_num, Size2f config_size = Size2f(640, 480), 
                                                float side_len = 30, bool varying_side_len = false) {
    if (obstacle_num <= 0) {
        cout << "The number of obstacles to be geenrated should be positive.\n";
        return {};
    }

    // By default, add 4 environment walls as obstacles
    vector<PolygonObstacle> res_obs_vec(obstacle_num + 4);
    vector<Point2f> top_obs_vertices{Point2f(0, 0), Point2f(config_size.width, 0), Point2f(config_size.width / 2, -10)},
                    bottom_obs_vertices{Point2f(0, config_size.height), Point2f(config_size.width, config_size.height), 
                                        Point2f(config_size.width / 2, config_size.height + 10)},
                    left_obs_vertices{Point2f(0, 0), Point2f(0, config_size.height), Point2f(-10, config_size.height / 2)},
                    right_obs_vertices{Point2f(config_size.width, 0), Point2f(config_size.width, config_size.height), 
                                        Point2f(config_size.width + 10, config_size.height / 2)};
    // -------0:top_obs------
    // |                    |
    // 2:left_obs       3:right_obs
    // |                    |
    // -----1:bottom_obs-----
    PolygonObstacle top_obs(top_obs_vertices), bottom_obs(bottom_obs_vertices), 
                    left_obs(left_obs_vertices), right_obs(right_obs_vertices);
    res_obs_vec[0] = top_obs;
    res_obs_vec[1] = bottom_obs;
    res_obs_vec[2] = left_obs;
    res_obs_vec[3] = right_obs;
    
    // Ensure vertices are placed in order.
    Eigen::Matrix<float, 2, 4> vertices_square, vertices_rectangle; 
                               vertices_square << -1.0 / 2, 1.0 / 2, 1.0 / 2, -1.0 / 2,
                                                  -1.0 / 2, -1.0 / 2, 1.0 / 2, 1.0 / 2;
                               vertices_rectangle << -1.0 / 2, 1.0 / 2, 1.0 / 2, -1.0 / 2,
                                                     -1.0 / 4, -1.0 / 4, 1.0 / 4, 1.0 / 4;
    Eigen::Matrix<float, 2, 3> vertices_triangle;                                                     
                               vertices_triangle << -1.0 / 2, 1.0 / 2, 0,
                                                    -1.0 * sqrt(3) / 6, -1.0 * sqrt(3) / 6, 1.0 * sqrt(3) / 3;

    vector<Eigen::MatrixXf> vertices_vec(3);
    vertices_vec[0] = vertices_square;
    vertices_vec[1] = vertices_rectangle;
    vertices_vec[2] = vertices_triangle;

    vector<Point2f> obs_center(obstacle_num);
    random_device rd_x, rd_y, rd_rotate_angle, rd_shape, rd_len;
    mt19937 rd_engine_x(rd_x()), rd_engine_y(rd_y()), rd_engine_rotate_angle(rd_rotate_angle()), rd_engine_shape(rd_shape());
    uniform_real_distribution<> distribution_x(0, config_size.width),
                                distribution_y(0, config_size.height),
                                distribution_rotate_angle(0, 2 * M_PI);
    // In 3D, only squares and rectangles are generated, use distribution_shape(0, 1). 
    // In 2D, triangles are also generated use (0, 2) in distribution_shape().
    uniform_int_distribution<> distribution_shape(0, 1), distribution_len(1, side_len);

    normal_distribution<float> normal_distribution_x(config_size.width / 2, config_size.width / 4),
                               normal_distribution_y(config_size.height / 2, config_size.height / 4);

    for (int i = 4; i < obstacle_num + 4; i++) {
        float x = distribution_x(rd_x), y = distribution_y(rd_y), angle = distribution_rotate_angle(rd_rotate_angle);
        int shape_type = distribution_shape(rd_shape);
        int len_type = distribution_len(rd_len);
        Point2f obs_center(x, y);
        Eigen::Matrix2f rotation;
                        rotation << cos(angle), -sin(angle), 
                                    sin(angle), cos(angle);
        Eigen::MatrixXf rotated_vertices;
        if  (!varying_side_len)
            rotated_vertices = rotation * vertices_vec[shape_type] * side_len;
        else
            rotated_vertices = rotation * vertices_vec[shape_type] * len_type;
        PolygonObstacle obs;
        for (int j = 0; j < rotated_vertices.cols(); j++) {
            Point2f cur_vertex(rotated_vertices(0, j) + obs_center.x,
                               rotated_vertices(1, j) + obs_center.y);
            obs.vertices.push_back(cur_vertex);
        }

        bool is_obs_valid = true;
        for (int j = 0; j < i; j++) {
            if (ObstaclesIntersect(obs, res_obs_vec[j])) {
                is_obs_valid = false;
                break;
            }
        }
        if (!is_obs_valid)
            i--;
        else
            res_obs_vec[i] = obs;
    }
    return res_obs_vec;
}

float ComputeObstacleArea(const PolygonObstacle& obs) {
    if (obs.vertices.size() <= 2) {
        throw std::invalid_argument("An obstacle with vertex number less than two is input in " + string(__func__));
    }
    float res = 0;
    for (int i = 0; i < obs.vertices.size(); i++)
        res += CrossProductVal(obs.vertices[i], obs.vertices[(i + 1) % obs.vertices.size()]);
    return abs(res) / 2;
}

float FreespaceArea(const vector<PolygonObstacle>& obstacles, const Size2f config_size, bool has_env_walls = true) {
    float total_obs_area = 0, total_area = config_size.width * config_size.height;
    int idx = has_env_walls ? 4 : 0;
    for ( ; idx < obstacles.size(); idx++)
        total_obs_area += ComputeObstacleArea(obstacles[idx]);
    return total_area - total_obs_area;
} 

Passages PureVisibilityPassageCheck(const vector<PolygonObstacle>& obstacles) {
    vector<vector<int>> res_psg_pair;
    vector<vector<Point2f>> res_psg_pts;
    
    // 4 if the first four wall obstacles are not considered.
    int start_idx = 4; 
    for (int i = start_idx; i < obstacles.size() - 1; i++) {
        int j = i < 4 ? 4 : i + 1;
        for (; j < obstacles.size(); j++) {
            vector<Point2f> psg_segment_pts = GetPassageSegmentPts(obstacles[i], obstacles[j]);
            
            bool is_psg_valid = true;
            for (int k = start_idx; k < obstacles.size(); k++) {
                if (k == i || k == j)
                    continue;
                if (!ObstacleFree(obstacles[k], psg_segment_pts[0], psg_segment_pts[1])) {
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

Passages ExtendedVisibilityPassageCheck(const vector<PolygonObstacle>& obstacles, 
                                        bool contain_env_walls = true,
                                        bool only_use_psg_segment = false) {
    vector<vector<int>> res_psg_pair;
    vector<vector<Point2f>> res_psg_pts;
    
    // 4 if the first four wall obstacles are not considered.
    int start_idx = contain_env_walls ? 0 : 4; 
    for (int i = start_idx; i < obstacles.size() - 1; i++) {
        int j = i < 4 ? 4 : i + 1;
        for (; j < obstacles.size(); j++) {
            vector<vector<Point2f>> psg_key_pts = SVIntersection(obstacles[i], obstacles[j]);
            vector<Point2f> psg_segment_pts = psg_key_pts.back();            
            // vector<Point2f> psg_segment_pts = GetPassageSegmentPts(obstacles[i], obstacles[j]);
            float psg_length = cv::norm(psg_segment_pts[0] - psg_segment_pts[1]);

            bool is_psg_valid = true;
            // k = 4 if walls are not considered at all, even in checking if it intersects with other passages.
            int k = 4;
            for (; k < obstacles.size(); k++) {
                if (k == i || k == j)
                    continue;
                
                if (only_use_psg_segment) {
                    if (!ObstacleFree(obstacles[k], psg_segment_pts[0], psg_segment_pts[1])) {
                        is_psg_valid = false;
                        break;
                    }
                }
                else {
                    if (!ObstacleFree(obstacles[k], psg_key_pts[0][0], psg_key_pts[0][1])
                        || !ObstacleFree(obstacles[k], psg_key_pts[1][0], psg_key_pts[1][1])) {
                        is_psg_valid = false;
                        break;
                    }
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

/// Only explicitly find passages linked to environment walls. Used for seperately processing environment walls.
Passages ExtendedVisibilityCheckForWalls(const vector<PolygonObstacle>& obstacles) {
    vector<vector<int>> res_psg_pair;
    vector<vector<Point2f>> res_psg_pts;
    
    // Find passage between walls and obstacles. Passage between walls are NOT detected.
    for (int i = 0; i < 4; i++) {
        for (int j = 4; j < obstacles.size(); j++) {
            vector<vector<Point2f>> psg_key_pts = SVIntersection(obstacles[i], obstacles[j]);
            vector<Point2f> psg_segment_pts = psg_key_pts.back();
            float psg_length = cv::norm(psg_segment_pts[0] - psg_segment_pts[1]);

            bool is_psg_valid = true;
            int k = 0;
            for (; k < obstacles.size(); k++) {
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

#endif