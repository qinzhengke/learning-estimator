#include "trase.hpp"
#include <fstream>
#include <cmath>
#include <tuple>
#include <random>

using namespace trase;
using namespace std;

struct Point{
    float x,y;
};

class World{
public:

};

class Robot{
public:

    Point pos;
    Point p0,p1={1,0},p2;
    default_random_engine rand_;

    Robot(){
        plan();
        plan();
        plan();
    }

    tuple<vector<float>,vector<float>> currPlanLine(){
        vector<float>x,y;
        const int n = 100;
        float dx = (p1.x - p0.x)/(float)n, dy = (p1.y - p0.y)/(float)n;
        for(int i=0; i<n+1; i++){
            x.push_back(p0.x+dx*(float)i);
            y.push_back(p0.y+dy*(float)i);
        }
        return tuple<vector<float>,vector<float>>(x,y);
    }

    Point move(){

    }

    Point plan_p1(Point p0){
        float d = M_PI/180.0 * 10.0;
        float a = atan2(p0.y, p0.x);
        float a_min = a+d, a_max=(a-d+2*M_PI);

        uniform_real_distribution<float> dist(a_min,a_max);
        float b = dist(rand_);
        return Point({cos(b), sin(b)});
    }

    Point plan_p2(Point p0, Point p1){

    }

    void plan(){
        p0 = p1;
        p1 = plan_p1(p0);
    }

};

int main() {

  // create figure and axis
  auto fig = figure();
  auto ax = fig->axis();

    const int n = 100;
    const float w = 2*M_PI/100;
    vector<float>x,y;
    for(int i=0; i<n+1; i++){
        float a = (float)(i%n)*w;
        x.push_back(cos(a));
        y.push_back(sin(a));
    }
    auto data = create_data().x(x).y(y);
    auto plt = ax->line(data);

    Robot walle;
    vector<float>walle_plan_x, walle_plan_y;
    tie(walle_plan_x, walle_plan_y) = walle.currPlanLine();
    auto walle_plan_data = create_data().x(walle_plan_x).y(walle_plan_y);
    auto walle_plan_plt = ax->line(walle_plan_data);

  // label axis
  ax->xlabel("x");
  ax->ylabel("y");

  BackendGL backend;
  fig->show(backend);
}