#include <chrono>
#include <ctime>
#include "../3d/RRTStar_3d.hpp"
#include "../vis/obs_path_vis.hpp"

using namespace std::chrono;

int main(int argc, char** argv) {
    ros::init(argc, argv, "obs_path_visualization");
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 40;
    float config_height = 400;
    vector<PolygonObstacle3d> obs_vec = GenerateRandomObstacles3d(obs_num, back_img.size(), config_height, 60, true);
    Point3f start = Point3f(1, 1, 100), end = Point3f(back_img.size().width - 1, back_img.size().height - 1, 200);
    RRTStarPlanner3d planner_len(start, end, obs_vec, 30, back_img.size(), 205, 0);
    RRTStarPlanner3d planner_mpw(start, end, obs_vec, 100, back_img.size(), 205, 1);
    RRTStarPlanner3d planner_gpw(start, end, obs_vec, 100, back_img.size(), 205, 2);
    RRTStarPlanner3d planner_len_constrain(start, end, obs_vec, 30, back_img.size(), 205, 0, true, 60);
    RRTStarPlanner3d planner_mpw_constrain(start, end, obs_vec, 100, back_img.size(), 205, 1, true, 60);
    RRTStarPlanner3d planner_gpw_constrain(start, end, obs_vec, 100, back_img.size(), 205, 2, true, 60);    
                  
    auto start_time = high_resolution_clock::now();
    bool planned = // planner_len.Plan() && planner_len_constrain.Plan(); 
                   planner_mpw.Plan() && planner_mpw_constrain.Plan() 
                   &&  planner_gpw.Plan() && planner_gpw_constrain.Plan(); 
    auto end_time = high_resolution_clock::now();
    auto duration_time = duration_cast<milliseconds>(end_time - start_time);
    float planning_time = (float)duration_time.count();

    cout << cv::norm(end - start) << "\n";
    if (planned) {
        cout << "Path is successfully planned\n";
        vector<Point3f> path_len = planner_len.GetPathInPts(), 
                        path_len_constrin = planner_len_constrain.GetPathInPts(), 
                        path_mpw = planner_mpw.GetPathInPts(),
                        path_mpw_constrain = planner_mpw_constrain.GetPathInPts(),
                        path_gpw = planner_gpw.GetPathInPts(),
                        path_gpw_constrain = planner_gpw_constrain.GetPathInPts();
        vector<vector<Point3f>> path_vec{path_mpw, path_mpw_constrain, path_gpw, path_gpw_constrain};
        VisualizeObstaclesAndPathsInTurn(obs_vec, path_vec);
    } 
    return 0;    
}