#include <chrono>
#include <ctime>
#include "../RRTStar.hpp"
#include "../PRMStar.hpp"
#include "path_processing.hpp"
#include <opencv2/viz/types.hpp>

using namespace std::chrono;

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 60;
    int obs_side_len = 60;
    bool varying_side_len = true;

    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), obs_side_len, varying_side_len);
    DrawObstacles(back_img, obs_vec, false);
    // Passages ev_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    Passages dg_check_res = PassageCheckDelaunayGraphWithWalls(obs_vec);
    for (int i = 0; i < dg_check_res.pts.size(); i++)
        DrawDashedLine(back_img, dg_check_res.pts[i][0], dg_check_res.pts[i][1], Scalar(0, 0, 0), 2, 20);
    
    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 1);
    float rrt_step = 50;
    RRTStarPlanner planner_len(start, end, obs_vec, rrt_step, back_img.size(), 0),
                   planner_mpw(start, end, obs_vec, rrt_step, back_img.size(), 2),
                   planner_gpw(start, end, obs_vec, rrt_step, back_img.size(), 3),
                   planner_constrained_len(start, end, obs_vec, rrt_step, back_img.size(), 0, true, 60),
                   planner_constrained_mpw(start, end, obs_vec, rrt_step, back_img.size(), 2, true, 200),
                   planner_constrained_gpw(start, end, obs_vec, rrt_step, back_img.size(), 3, true, 200),
                   planner_compound_mpw(start, end, obs_vec, rrt_step, back_img.size(), 4),
                   planner_compound_gpw(start, end, obs_vec, rrt_step, back_img.size(), 5);
    /* PRMStarPlanner planner_prm(start, end, obs_vec, back_img.size(), 3);
    planner_prm.QueryPath(back_img);
    DrawPath(back_img, planner_prm.GetPath(), cv::viz::Color::blue());
    imshow("PRM* PTOPP", back_img);
    waitKey(0);     
    return 1;*/
                  
    auto start_time = high_resolution_clock::now();
    bool planned_mpw = planner_mpw.Plan(back_img);
    bool planned_gpw = planner_gpw.Plan(back_img);
    // bool planned_constrained_len = planner_constrained_len.Plan(back_img);
    bool planned_constrained_mpw = planner_constrained_mpw.Plan(back_img);
    bool planned_constrained_gpw = planner_constrained_gpw.Plan(back_img);
    // bool planned_compound_mpw = planner_compound_mpw.Plan(back_img);
    // bool planned_compound_gpw = planner_compound_gpw.Plan(back_img); 
    auto end_time = high_resolution_clock::now();
    auto duration_time = duration_cast<milliseconds>(end_time - start_time);
    float ptopp_planning_time = (float)duration_time.count() / 2;

    start_time = high_resolution_clock::now();
    bool planned_len = planner_len.Plan(back_img);
    end_time = high_resolution_clock::now();
    duration_time = duration_cast<milliseconds>(end_time - start_time);
    float len_planning_time = (float) duration_time.count();

    cout << "Planning time in brute-force ptopp: " << ptopp_planning_time << " ms"
         << "\nPlanning time in shorest length planning: " << len_planning_time << " ms\n";

    vector<PathNode*> planned_path_len, planned_path_mpw, planned_path_gpw, 
                      planned_path_constrained_len, planned_path_constrained_mpw, planned_path_constrained_gpw,
                      planned_path_compound_mpw, planned_path_compound_gpw;
    if (planned_len) {
        planned_path_len = planner_len.GetPath();
        planned_path_mpw = planner_mpw.GetPath();
        planned_path_gpw = planner_gpw.GetPath();
        // planned_path_constrained_len = planner_constrained_len.GetPath();
        planned_path_constrained_mpw = planner_constrained_mpw.GetPath();
        planned_path_constrained_gpw = planner_constrained_gpw.GetPath();        
        // planned_path_compound_mpw = planner_compound_mpw.GetPath();
        // planned_path_compound_gpw = planner_compound_gpw.GetPath();
        // auto smooth_path_len = QuadraticBSplineSmoothing(planned_path_len);
        // auto smooth_path_mpw = QuadraticBSplineSmoothing(planned_path_mpw);
        // auto smooth_path_gpw = QuadraticBSplineSmoothing(planned_path_gpw);
        // auto smooth_path_compound_mpw = QuadraticBSplineSmoothing(planned_path_compound_mpw);
        // auto smooth_path_compound_gpw = QuadraticBSplineSmoothing(planned_path_compound_gpw);
        // DrawDashedPath(back_img, planned_path_len, cv::viz::Color(0.357, 0.122, 1), 3);
        DrawDashedPath(back_img, planned_path_mpw, cv::viz::Color(0.424, 0.804, 0), 3);
        DrawDashedPath(back_img, planned_path_gpw, cv::viz::Color(0.871, 0.604, 0), 3);
        // DrawPath(back_img, planned_path_constrained_len, cv::viz::Color(0.357, 0.122, 1), 3);
        DrawPath(back_img, planned_path_constrained_mpw, cv::viz::Color(0.424, 0.804, 0), 3);
        DrawPath(back_img, planned_path_constrained_gpw, cv::viz::Color(0.871, 0.604, 0), 3);
        // DrawPath(back_img, smooth_path_compound_mpw, cv::viz::Color(0.729, 0.345, 0.686));
        // DrawPath(back_img, smooth_path_compound_gpw, cv::viz::Color(0.118, 0.776, 1));

        auto passed_psgs_mpw = RetrievePassedPassages(planned_path_mpw, planner_mpw.passage_pts_),
             passed_psgs_gpw = RetrievePassedPassages(planned_path_gpw, planner_gpw.passage_pts_);

        cout << "sorted passage width list in len:\n";
        for(float width : planner_len.target_node_->sorted_passage_list)
            cout << width << ", ";
        cout << "\nsorted passage width list in constrained len:\n";
        for(float width : planner_constrained_len.target_node_->sorted_passage_list)
            cout << (width < 0 ? -1 / width : width) << ", "; 

        cout << "\nsorted passage width list in mpw:\n";
        for(float width : planner_mpw.target_node_->sorted_passage_list)
            cout << width << ", ";
        cout << "\nsorted passage width list in constrained mpw:\n";
        for(float width : planner_constrained_mpw.target_node_->sorted_passage_list)
           cout << (width < 0 ? -1 / width : width) << ", "; 

        cout << "\nsorted passage width list in gpw:\n";
        for(float width : planner_gpw.target_node_->sorted_passage_list)
            cout << width << ", ";
        cout << "\nsorted passage width list in constrained gpw:\n";
        for(float width : planner_constrained_gpw.target_node_->sorted_passage_list)
            cout << (width < 0 ? -1 / width : width) << ", ";                

        cout << "\npassage widths in mpw:\n";
        for (int psg_idx : passed_psgs_mpw.first) {
            auto psg_pts = planner_mpw.passage_pts_[psg_idx];
            cout << cv::norm(psg_pts[0] - psg_pts[1]) << ", ";
        }
        cout << "\npassage widths in gpw:\n";        
        for (int psg_idx : passed_psgs_gpw.first) {
            auto psg_pts = planner_gpw.passage_pts_[psg_idx];
            cout << cv::norm(psg_pts[0] - psg_pts[1]) << ", ";
        }   
        cout << "\n"; 
    }

    for (int i = 4; i < obs_num + 4; i++) {
        // Point2f cur_centroid = GetObstaclesCentroids({obs_vec[i]}).front();
        vector<vector<Point>> input_array_cv(1, vector<Point>(obs_vec[i].vertices.begin(), obs_vec[i].vertices.end()));
        fillPoly(back_img, input_array_cv, Scalar(0.827, 0.827, 0.827));
        int cur_obs_vertex_num = obs_vec[i].vertices.size();
        for (int j = 0; j < cur_obs_vertex_num; j++)
            line(back_img, obs_vec[i].vertices[j], obs_vec[i].vertices[(j + 1) % cur_obs_vertex_num], Scalar(0, 0, 0), 2);   
        // putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);         
    }    
    string save_directory = "./src/ptopp/src/img/planning_img/";
    string file_name_postfix, file_name_prefix, file_name;
    file_name_prefix = "Obs_Num_" + to_string(obs_num) + "_Side_" + to_string(obs_side_len) + (varying_side_len ? "_varying_len_" : "fixed_len_") + 
                        "Sample_" + to_string(planner_len.MAX_GRAPH_SIZE) + "_Constrained_";
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = to_string(cur_tm->tm_year + 1900) + "-"
                        + to_string(cur_tm->tm_mon + 1) + "-"
                        + to_string(cur_tm->tm_mday) + "_"
                        + to_string(cur_tm->tm_hour) + "-"
                        + to_string(cur_tm->tm_min) + ".png";
    file_name = file_name_prefix + file_name_postfix;

    while (true) {
        imshow("RRT* PTOPP", back_img);
        int key_input = (waitKey(50) & 0xFF);
        if (key_input == 's')
            imwrite(save_directory + file_name, 255 * back_img);
        else if (key_input == 'q')
            break;
        // cout << key_input << "\n";
    }
    return 0;    
}