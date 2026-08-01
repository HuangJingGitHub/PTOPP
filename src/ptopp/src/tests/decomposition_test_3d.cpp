#include <chrono>
#include <ctime>
#include <fstream>
#include "../3d/RRTStar_3d.hpp"
#include "../decomposition.hpp"

int main(int argc, char** argv) {
    Mat back_img(Size(1000, 600), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 20;
    float config_height = 400, side_len = 60;
    bool varying_side_len = true;

    vector<PolygonObstacle3d> obs_vec = GenerateRandomObstacles3d(obs_num, back_img.size(), config_height, side_len, varying_side_len);
    vector<PolygonObstacle> obs_vec_2d = ConvertTo2dObstacles(obs_vec);
    Passages passages_2d = PassageCheckDelaunayGraphWithWalls(obs_vec_2d);

    vector<vector<int>> cells = ReportGabrielCells(obs_vec_2d, passages_2d.pairs, true);
    vector<PolygonCell> cells_2d = GetGabrielCellsInfo(cells, passages_2d);
    int cell_cnt = 0;
    for (auto& cell : cells_2d) {
        cout << cell_cnt++ << ": ";
        for (int obs_idx : cell.obs_indices)    
            cout << obs_idx << ", ";
        cout << "\n";
    }
    
    Passages3d passages = PassageCheckInDelaunayGraph3d(obs_vec);
    FilterPassagesWithWalls3d(passages);
    Passages3d passages_wall = PassageCheckForWalls3d(obs_vec);
    Passages3d passages_3d = PassageCheckDelaunayGraphWithWalls3d(obs_vec);
    cout << "passages without walls: \n";
    for (auto psg : passages.pairs)
        cout << psg[0] << "-" << psg[1] << ", ";
    cout << "\npassages of walls: \n";
    for (auto psg : passages_wall.pairs)
        cout << psg[0] << "-" << psg[1] << ", ";  
    cout << "\nall passages in 3d: \n";
    for (auto psg : passages_3d.pairs)
        cout << psg[0] << "-" << psg[1] << ", ";

    for (int i = 4; i < obs_num + 4; i++) {
        Point2f cur_centroid = GetObstaclesCentroids({obs_vec_2d[i]}).front();
        vector<vector<Point>> input_array_cv(1, vector<Point>(obs_vec_2d[i].vertices.begin(), obs_vec_2d[i].vertices.end()));
        fillPoly(back_img, input_array_cv, Scalar(0.827, 0.827, 0.827));
        int obs_vertex_num = obs_vec_2d[i].vertices.size();
        for (int j = 0; j < obs_vertex_num; j++)
            line(back_img, obs_vec_2d[i].vertices[j], obs_vec_2d[i].vertices[(j + 1) % obs_vertex_num], Scalar(0, 0, 0), 2);   
        putText(back_img, std::to_string(i), cur_centroid - Point2f(10, -5), cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);         
    }
    for (int i = 0; i < passages.pts.size(); i++) {
        if (passages.heights[i][0] < 0.1)
            DrawDashedLine(back_img, passages.pts[i][0], passages.pts[i][1]);
        else
            DrawDashedLine(back_img, passages.pts[i][0], passages.pts[i][1], Scalar(0, 0, 255));
    }
    for (int i = 0; i < passages_wall.pts.size(); i++) {
        if (passages_wall.heights[i][0] < 0.1)
            DrawDashedLine(back_img, passages_wall.pts[i][0], passages_wall.pts[i][1]);
        else    
            DrawDashedLine(back_img, passages_wall.pts[i][0], passages_wall.pts[i][1], Scalar(0, 0, 255));        
    }

    Point2f start_pt(1, 1), end_pt(999, 599);
    DrawDashedLine(back_img, start_pt, end_pt, Scalar(255, 0, 0));
    vector<vector<int>> traverse_res = GetPassedCellsAndObstacles(start_pt, end_pt, cells_2d, obs_vec_2d, back_img);
    cout << "\nPassed cell indices: \n";
    for (int idx : traverse_res[0])
        cout << idx << ", ";
    cout << "\nPassed obstacle indices: \n";
    for (int idx : traverse_res[1])
        cout << idx << ", ";
    cout << "\n";

    //vector<PolygonCell3d> cells_3d = GetCompoundGabrielCells3d(obs_vec, passages_3d, back_img);
    /*for (PolygonCell3d& cell : cells_3d) {
        cout << "\n3d cell: \n";
        for (int obs_idx : cell.obs_indices)
            cout << obs_idx << ", ";
        cout << "\n";
        for (vector<int> intersect_psg : cell.intersect_psg_pairs)
            cout << intersect_psg[0] << "-" << intersect_psg[1] << ", ";
    }*/

    imshow("Obstacles on Base Ground", back_img);
    waitKey(0);
    return 0;    
}