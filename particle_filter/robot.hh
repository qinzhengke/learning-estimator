#include "../util/render.hh"
#include <vector>
#include <random>
using namespace std;

struct State{

    Point p;

    double yaw;

    static default_random_engine rand_;

    void addNoise(double level){
        uniform_real_distribution<double> dist(-level, level);
        p.x += dist(rand_);
        dist = uniform_real_distribution<double>(-level, level);
        p.y += dist(rand_);
    }

    render_obj_t renderObj(double r, string color){

        vector<double>s1_x,s1_y;
        for(int i=0; i<9; i++){
            double a = (double)i * M_PI / 4.0 + M_PI / 8.0 + yaw;
            Point p1 = {cos(a)*r + p.x, sin(a)*r + p.y};
            s1_x.push_back(p1.x);
            s1_y.push_back(p1.y);
        }

        vector<double>s2_x, s2_y;
        s2_x.push_back(p.x);
        s2_y.push_back(p.y);
        s2_x.push_back(cos(yaw)*cos(M_PI/8)*r+p.x);
        s2_y.push_back(sin(yaw)*cos(M_PI/8)*r+p.y);

        return render_obj_t {{ {s1_x,s1_y,color}, {s2_x,s2_y,color} }};
    }

};
default_random_engine State::rand_;

struct Robot{

    State curr_state_;
    vector<State> future_state_;
    double a0_,a1_=0,a2_=M_PI/2.0;
    Point q0_,q1_={cos(a1_),sin(a1_)};
    static default_random_engine rand_;

    Robot(){ }
    Robot(State st){
        curr_state_ = st;
    }

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
            double x = q0_.x + dx*(double)i;
            double y = q0_.y + dy*(double)i;
            double yaw = atan2(q2.y-q0_.y, q2.x-q0_.x);

            states.push_back({x,y,yaw});
        }

        // Bezier curve.
        for(int i=0; i<n; i++){
            double t = (double)i/(double)n;
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
};

default_random_engine Robot::rand_;