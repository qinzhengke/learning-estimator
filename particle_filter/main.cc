#include "trase.hpp"
#include <fstream>
#include <cmath>
#include <tuple>
#include <random>

using namespace trase;
using namespace std;

using XYTuple = tuple<vector<float>,vector<float>>;

struct Point{
    float x,y;
};

class World{
public:

};

class Robot{
public:

    Point pos_;
    float a0_,a1_=0,a2_=M_PI/2.0;
    Point q0_,q1_={cos(a1_),sin(a1_)};
    vector<float> exe_curve_x_, exe_curve_y_;
    default_random_engine rand_;

    Robot(){ }

    XYTuple currPlanLine(){
        vector<float>x,y;
        const int n = 100;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)};
        float dx = (p1.x - p0.x)/(float)n, dy = (p1.y - p0.y)/(float)n;
        for(int i=0; i<n+1; i++){
            x.push_back(p0.x+dx*(float)i);
            y.push_back(p0.y+dy*(float)i);
        }

        return XYTuple(x,y);
    }

    XYTuple currPlanCurve(){
        vector<float>x,y;
        const int n=5;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)};
        Point q2 = {p0.x+(p1.x-p0.x)*0.8f, p0.y+(p1.y-p0.y)*0.8f};
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
        return XYTuple(x,y);
    }

    void move(){
        if(exe_curve_x_.empty()){
            tie(exe_curve_x_,exe_curve_y_) = plan();
        }

        pos_ = {exe_curve_x_.front(), exe_curve_y_.front()};
        exe_curve_x_.erase(exe_curve_x_.begin());
        exe_curve_y_.erase(exe_curve_y_.begin());
        printf("exe_curve:%lu,x:%.4f,y:%.4f\n",exe_curve_x_.size(), pos_.x, pos_.y);
    }



    XYTuple plan(){
        printf("replan path!\n");
        a0_ = a1_;
        a1_ = a2_;
        float d = M_PI/180.0 * 60.0;
        float a2_min =  a1_ + d, a2_max = a1_ + M_PI;
        uniform_real_distribution<float> dist2(a2_min,a2_max);
        a2_ = dist2(rand_);

        q0_ = q1_;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)}, p2={cos(a2_),sin(a2_)};
        q1_ = {p1.x+(p2.x-p1.x)*0.2f, p1.y+(p2.y-p1.y)*0.2f};

        return currPlanCurve();
    }

    XYTuple draw(){

        const int n = 20;
        const float w = 2*M_PI/(float)n, r=0.05;
        vector<float>shape_x,shape_y;
        for(int i=0; i<n+1; i++){
            float a = (float)(i%n)*w;
            shape_x.push_back(cos(a)*r+pos_.x);
            shape_y.push_back(sin(a)*r+pos_.y);
        }
        printf("center:%.4f,%.4f\n", pos_.x, pos_.y);
        return XYTuple(shape_x,shape_y);
    }

};

int main() {

  // create figure and axis
  auto fig = figure({800,800});
  auto ax = fig->axis();

    const int n = 100;
    const float w = 2*M_PI/100;
    vector<float>world_x,world_y;
    for(int i=0; i<n+1; i++){
        float a = (float)(i%n)*w;
        world_x.push_back(cos(a));
        world_y.push_back(sin(a));
    }
    auto data = create_data().x(world_x).y(world_y);
    auto plt = ax->line(data);

    Robot walle;
    vector<float>walle_plan_x, walle_plan_y;
    // for (int i=0; i<3; i++){
    //     walle.plan();
    //     vector<float>x, y;

    //     tie(x, y) = walle.currPlanCurve();
    //     for(auto xx : x){ walle_plan_x.push_back(xx); }
    //     for(auto yy : y){ walle_plan_y.push_back(yy); }
    // }
    // auto walle_plan_data = create_data().x(walle_plan_x).y(walle_plan_y);
    // auto walle_plan_plt = ax->line(walle_plan_data);
    // auto walle_plan_plt = ax->points(walle_plan_data);

    shared_ptr<Geometry> walle_render = nullptr;
    vector<float>walle_shape_x, walle_shape_y;
    for(int i=0; i<1000; i++){
        walle.move();        
        tie(walle_shape_x,walle_shape_y) = walle.draw();
        auto walle_shape_data = create_data().x(walle_shape_x).y(walle_shape_y);
        // auto walle_shape_plt = ax->line(walle_shape_data);
        if(walle_render == nullptr){
            walle_render = ax->line(walle_shape_data);
        }
        else{
            walle_render->add_frame(walle_shape_data, i*0.5);
        }
    }

  // label axis
  ax->xlabel("x");
  ax->ylabel("y");

  BackendGL backend;
  fig->show(backend);
}