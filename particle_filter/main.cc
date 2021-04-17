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
    float a0_,a1_=0,a2_=M_PI/2.0;
    Point q0_,q1_={cos(a1_),sin(a1_)};
    default_random_engine rand_;

    Robot(){
    }

    tuple<vector<float>,vector<float>> currPlanLine(){
        vector<float>x,y;
        const int n = 100;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)};
        float dx = (p1.x - p0.x)/(float)n, dy = (p1.y - p0.y)/(float)n;
        for(int i=0; i<n+1; i++){
            x.push_back(p0.x+dx*(float)i);
            y.push_back(p0.y+dy*(float)i);
        }

        // Point p2 = {cos(a2_), sin(a2_)};
        // dx = (p2.x-p1.x)/(float)(n), dy = (p2.y-p1.y)/(float)n;

        // for(int i=0; i<n+1; i++){
        //     x.push_back(p1.x+dx*(float)i);
        //     y.push_back(p1.y+dy*(float)i);
        // }
        return tuple<vector<float>,vector<float>>(x,y);
    }

    tuple<vector<float>,vector<float>> currPlanCurve(){
        vector<float>x,y;
        const int n=100;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)};
        Point q2 = {p0.x+(p1.x-p0.x)*0.2f, p0.y+(p1.y-p0.y)*0.2f};
        float dx = (q2.x - q0_.x)/(float)n, dy = (q2.y - q0_.y)/(float)n;
        for(int i=0; i<n; i++){
            float t = (float)i/(float)n;
            x.push_back(q0_.x + dx*(float)i);
            y.push_back(q0_.y + dy*(float)i);
        }

        for(int i=0; i<n; i++){
            float t = (float)i/(float)n;
            x.push_back((1.0-t)*(1.0-t)*q2.x + 2.0*t*(1.0-t)*p1.x + t*t*q1_.x);
            y.push_back((1.0-t)*(1.0-t)*q2.y + 2.0*t*(1.0-t)*p1.y + t*t*q1_.y);
        }
        return tuple<vector<float>,vector<float>>(x,y);
    }

    Point move(){

        Point pt;
        return pt;
    }


    void plan(){
        a0_ = a1_;
        a1_ = a2_;
        float d = M_PI/180.0 * 60.0;
        float a2_min =  a1_ + d, a2_max = a1_ + M_PI;
        uniform_real_distribution<float> dist2(a2_min,a2_max);
        a2_ = dist2(rand_);

        q0_ = q1_;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)}, p2={cos(a2_),sin(a2_)};
        q1_ = {p1.x+(p2.x-p1.x)*0.2f, p1.y+(p2.y-p1.y)*0.2f};

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
    for (int i=0; i<20; i++){
        walle.plan();
        vector<float>walle_plan_x, walle_plan_y;
        // tie(walle_plan_x, walle_plan_y) = walle.currPlanLine();
        // auto walle_plan_data = create_data().x(walle_plan_x).y(walle_plan_y);
        // auto walle_plan_plt = ax->line(walle_plan_data);

        tie(walle_plan_x, walle_plan_y) = walle.currPlanCurve();
        auto walle_plan_data = create_data().x(walle_plan_x).y(walle_plan_y);
        auto walle_plan_plt = ax->line(walle_plan_data);
        // plt = ax
        // plt->add_frame(walle_plan_data, i+1);
    }

  // label axis
  ax->xlabel("x");
  ax->ylabel("y");

  BackendGL backend;
  fig->show(backend);
}