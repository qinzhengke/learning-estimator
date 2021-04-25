#include <fstream>
#include <cmath>
#include <random>
#include <matplot/matplot.h>
#include "../util/render.hh"
#include "robot.hh"

using namespace std;

struct World{

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


struct SensorGroup{
    // Sensor numbers.
    constexpr static int n = 6;
    Point sensors[n];

    SensorGroup(){
        double d = M_PI * 2.0 / n;
        for(int i=0; i<n; i++){
            sensors[i].x = cos(d*i);
            sensors[i].y = sin(d*i);
        }
    }


    double distance(Point p1, Point p2){
        return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
    }

    double measurement(State st){
        double d_min = 9999.0;
        for(int i=0; i<n; i++){
            double d = distance(st.p, sensors[i]);
            d_min = d_min < d ? d : d_min;
        }
        return d_min;
    }

    render_obj_t renderObj(){
        vector<double> x,y;
        for(int i=0; i<n; i++){
            x.push_back(sensors[i].x);
            y.push_back(sensors[i].y);
        }
        return render_obj_t { {{x,y,"r",SCATTER}} };
    }
};

struct Particle{

    // State
    State state_;

    // Measurement
    double dist_;

    double weight_;

    static default_random_engine rand_;

    void updateMeasurement(double d){
        dist_ = d;
    }

    // Update the weight of this particle base on the measurement.
    void updateWeight(double sensor_dist){
        double err = dist_ - sensor_dist;
        double sigma2 = 0.9*0.9;
        weight_ = exp(err*err / 2.0 / sigma2);
    }

    static Particle randomCreate(double x_min, double x_max, double y_min, double y_max){

        Particle p;
        auto dist = uniform_real_distribution<double>(x_min,x_max);
        p.state_.p.x = dist(rand_);
        dist = uniform_real_distribution<double>(y_min,y_max);
        p.state_.p.y = dist(rand_);
        dist = uniform_real_distribution<double>(0, M_PI*2);
        p.state_.yaw = dist(rand_);
        return p;
    }

};
default_random_engine Particle::rand_;

struct Estimator{

    static constexpr int n = 50;

    vector<Particle> particles_;
    vector<double> weight_pdf_;

    void resample(){
        if(particles_.size() == 0){
            for(int i=0; i<n; i++){
                auto p = Particle::randomCreate(-1,1,-1,1);
                particles_.push_back(p);
                weight_pdf_.push_back(0);
            }
        }
        else{

            // Update weight PDF
            double accum_weight = 0;
            for(int i=0; i<particles_.size(); i++){
                accum_weight += particles_[i].weight_;
                weight_pdf_[i] = accum_weight;
            }
            
            vector<Particle> new_particles;
            for(int i=0; i<particles_.size(); i++){
                uniform_real_distribution<double> uniform(0, accum_weight);
                double w = uniform(Particle::rand_);
                int index =std::lower_bound(weight_pdf_.begin(), weight_pdf_.end(), w) - weight_pdf_.begin();
                Particle p;
                p.state_ = particles_[index].state_;
                p.state_.addNoise(0.05);
                new_particles.push_back(p);
            }

            particles_ = new_particles;
        }
    }

    void updateDistribution(){
    } 

    State output(){
        double m_x=0,m_y=0,m_cnt=0;
        for(auto p : particles_){
            m_cnt += p.weight_;
            m_x += p.state_.p.x;
            m_y += p.state_.p.y;
        }
        return State({{m_x/m_cnt, m_y/m_cnt},0});
    }
};

int main() {

    Robot walle;
    World world;
    SensorGroup sensor_group;
    Estimator estimator;
    Renderer rdr;

    for(;;){
        walle.move();        


        // Step 1: Get the measurement.
        double real_mea = sensor_group.measurement(walle.curr_state_);

        estimator.resample();

        // Step 2: Update the weigth of particles.
        for(auto &p : estimator.particles_){
            p.updateMeasurement(sensor_group.measurement(p.state_));
            p.updateWeight(real_mea);
        }

        State st_out = estimator.output();


        // Draw all the entities.
        rdr.reset();
        rdr.addRenderObj(world.renderObj());
        rdr.addRenderObj(walle.curr_state_.renderObj(0.1, "r"));
        rdr.addRenderObj(sensor_group.renderObj());
        for(auto p: estimator.particles_){
            rdr.addRenderObj(p.state_.renderObj(0.05, "b"));
        }
        rdr.addRenderObj(st_out.renderObj(0.1, "g"));
        rdr.render();
        getchar();
    }
}