#include<iostream>
#include<stack>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define INF 10000
#define MAX_VERTEX_COUNT 35


class Floyd{
public:
    Mat weight_graph;
    int node_count;

    int arrDis[MAX_VERTEX_COUNT][MAX_VERTEX_COUNT];
    int arrPath[MAX_VERTEX_COUNT][MAX_VERTEX_COUNT];

    vector<int> path;

    Floyd(){};
    void loadMatrix(Mat& matrix);
    void initFloydGraph();
    void calcPath(int start,int endl);
    void updateFloydGraph(int i,int j,int value);
    void printPath();
};
