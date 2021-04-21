#include <fstream>
#include <cmath>
#include <tuple>
#include <random>
#include <matplot/matplot.h>
#include "../util/render.hh"

using namespace std;

using XYTuple = tuple<vector<double>,vector<double>>;

class World{
public:

    vector<double>world_x,world_y;
    World(){
        const int n = 100;
        const double w = 2*M_PI/n;
        for(int i=0; i<n+1; i++){
            double a = (double)(i%n)*w;
            world_x.push_back(cos(a));
            world_y.push_back(sin(a));
        }
    }

    render_obj_t renderObj(){
        return render_obj_t {{{world_x,world_y,"b"}}};
    }
};

struct State{
    Point p;
    double yaw;
};

class Robot{
public:

    // Point pos_;
    // double yaw;
    State curr_state_;
    vector<State> future_state_;
    double a0_,a1_=0,a2_=M_PI/2.0;
    Point q0_,q1_={cos(a1_),sin(a1_)};
    // vector<double> exe_curve_x_, exe_curve_y_;
    default_random_engine rand_;

    Robot(){ }

    XYTuple currPlanLine(){
        vector<double>x,y;
        const int n = 100;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)};
        double dx = (p1.x - p0.x)/(double)n, dy = (p1.y - p0.y)/(double)n;
        for(int i=0; i<n+1; i++){
            x.push_back(p0.x+dx*(double)i);
            y.push_back(p0.y+dy*(double)i);
        }

        return XYTuple(x,y);
    }

    vector<State> currPlanCurve(){
        vector<double>x,y;
        const int n=5;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)};
        Point q2 = {p0.x+(p1.x-p0.x)*0.8f, p0.y+(p1.y-p0.y)*0.8f};
        double dx = (q2.x - q0_.x)/(double)n, dy = (q2.y - q0_.y)/(double)n;
        vector<State> states;
        for(int i=0; i<n; i++){
            double t = (double)i/(double)n;
            // x.push_back(q0_.x + dx*(double)i);
            // y.push_back(q0_.y + dy*(double)i);
            double x = q0_.x + dx*(double)i;
            double y = q0_.y + dy*(double)i;
            double yaw = atan2(q2.y-q0_.y, q2.x-q0_.x);

            states.push_back({x,y,yaw});
        }

        // Bezier curve.
        for(int i=0; i<n; i++){
            double t = (double)i/(double)n;
            // x.push_back((1.0-t)*(1.0-t)*q2.x + 2.0*t*(1.0-t)*p1.x + t*t*q1_.x);
            // y.push_back((1.0-t)*(1.0-t)*q2.y + 2.0*t*(1.0-t)*p1.y + t*t*q1_.y);
            double x = (1.0-t)*(1.0-t)*q2.x + 2.0*t*(1.0-t)*p1.x + t*t*q1_.x;
            double y = (1.0-t)*(1.0-t)*q2.y + 2.0*t*(1.0-t)*p1.y + t*t*q1_.y;
            Point m1 = {(1-t)*q2.x + t*p1.x, (1-t)*q2.y + t*p1.y};
            Point m2 = {(1-t)*p1.x + t*q1_.x, (1-t)*p1.y + t*q1_.y};
            double yaw = atan2(m2.y-m1.y, m2.x-m1.x);
            states.push_back({x,y,yaw});
        }
        return states;
    }

    void move(){
        if(future_state_.empty()){
            future_state_ = plan();
        }

        // pos_ = {exe_curve_x_.front(), exe_curve_y_.front()};
        // exe_curve_x_.erase(exe_curve_x_.begin());
        // exe_curve_y_.erase(exe_curve_y_.begin());
        curr_state_ = future_state_.front();
        future_state_.erase(future_state_.begin());
        // printf("exe_curve:%lu,x:%.4f,y:%.4f\n",exe_curve_x_.size(), pos_.x, pos_.y);
    }

    vector<State> plan(){
        printf("replan path!\n");
        a0_ = a1_;
        a1_ = a2_;
        double d = M_PI/180.0 * 60.0;
        double a2_min =  a1_ + d, a2_max = a1_ + M_PI;
        uniform_real_distribution<double> dist2(a2_min,a2_max);
        a2_ = dist2(rand_);

        q0_ = q1_;
        Point p0 = {cos(a0_),sin(a0_)}, p1={cos(a1_),sin(a1_)}, p2={cos(a2_),sin(a2_)};
        q1_ = {p1.x+(p2.x-p1.x)*0.2f, p1.y+(p2.y-p1.y)*0.2f};

        return currPlanCurve();
    }

    render_obj_t renderObj(){

        const int n = 20;
        const double w = 2*M_PI/(double)n, r=0.05;
        vector<double>s1_x,s1_y;
        for(int i=0; i<9; i++){
            double a = (double)i * M_PI / 4.0 + M_PI / 8.0;
            Point p1 = {cos(a)*r + curr_state_.p.x, cos(a)*r + curr_state_.p.y};
            // Point p2 = {cos(a+M_PI/4.0)*r + curr_state_.p.x, cos(a+M_PI/4.0)*r + curr_state_.p.y};
            s1_x.push_back(p1.x);
            // s1_x.push_back(p2.x);
            s1_y.push_back(p1.y);
            // s1_y.push_back(p2.y);
        }

        vector<double>s2_x, s2_y;
        s2_x.push_back(curr_state_.p.x);
        s2_y.push_back(curr_state_.p.y);
        s2_x.push_back(cos(curr_state_.yaw)*cos(M_PI/8)*r+curr_state_.p.x);
        s2_y.push_back(sin(curr_state_.yaw)*cos(M_PI/8)*r+curr_state_.p.y);

        return render_obj_t {{ {s1_x,s2_y,"r"}, {s2_x,s2_y,"r"} }};
    }

};

int main() {

    Robot walle;
    World world;

    // for (int i=0; i<3; i++){
    //     walle.plan();
    //     vector<double>x, y;

    //     tie(x, y) = walle.currPlanCurve();
    //     for(auto xx : x){ walle_plan_x.push_back(xx); }
    //     for(auto yy : y){ walle_plan_y.push_back(yy); }
    // }
    // auto walle_plan_data = create_data().x(walle_plan_x).y(walle_plan_y);
    // auto walle_plan_plt = ax->line(walle_plan_data);
    // auto walle_plan_plt = ax->points(walle_plan_data);

    // shared_ptr<Geometry> walle_render = nullptr;

    Renderer rdr;
    vector<double>walle_shape_x, walle_shape_y;
    for(int i=0; i<1000; i++){
        walle.move();        
        rdr.reset();
        rdr.addRenderObj(world.renderObj());
        rdr.addRenderObj(walle.renderObj());
        rdr.render();
        getchar();
    }
}