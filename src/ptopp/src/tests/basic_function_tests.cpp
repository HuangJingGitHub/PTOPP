#include "../obstacles.hpp"

int main(int argc, char** argv) {
    Point2f psg_pt_1(159.147, 330.385), psg_pt_2(182.561, 217.397), 
            path_pt_1(175.447, 253.223), path_pt_2(175.058, 253.837), path_pt_3(174.658, 254.446);
    if (SegmentIntersection(path_pt_1, path_pt_2, psg_pt_1, psg_pt_2))
        cout << "First intersection\n";
    if (SegmentIntersection(path_pt_2, path_pt_3, psg_pt_1, psg_pt_2))
        cout << "Second intersection\n";    
    return 0;
}