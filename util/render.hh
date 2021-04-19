#include "../3rd-party/matplotplusplus-1.0.1/source/matplot/matplot.h"

using namespace std;

struct Point{
    double x,y;
};

struct render_part_t{
    vector<double>x,y;
    string color="b";
};

struct render_obj_t{
    vector<render_part_t> parts;
};

class Renderer{
public:
    vector<render_obj_t> objs_;
    matplot::figure_handle fig_;
    matplot::axes_handle ax_;
    Renderer(){
        fig_ = matplot::figure();
        ax_ = fig_->add_axes();
     }
    void reset(){
        objs_.clear();
    }
    void addRenderObj(render_obj_t obj){
        objs_.push_back(obj);
    }

    void render(){

        ax_->clear();
        for(int i=0; i<objs_.size(); i++){
            for(auto p : objs_[i].parts){
                ax_->plot(p.x, p.y, p.color.c_str());
                matplot::hold(matplot::on);
            }
        }
        matplot::axis(matplot::equal);
        matplot::show();
    }

};