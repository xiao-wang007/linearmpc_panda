
#include <vector>
#include <string>
#include <Eigen/Dense>
//#include "../include/linearmpc_panda/myutils.h"
#include <linearmpc_panda/myutils.h>
#include <fstream>
// #include <iomanip>
#include <stdexcept>

//#################################################################################
Eigen::VectorXd cumulative_sum(const Eigen::VectorXd& vec) {
    Eigen::VectorXd result(vec.size());
    double acc = 0.0;
    for (int i = 0; i < vec.size(); ++i) {
        acc += vec(i);
        result(i) = acc;
    }
    return result;
}

//#################################################################################
template <typename T>
class LinearSpline {
public:
    LinearSpline(std::vector<double> ts, std::vector<T> ys)
        : knots_(std::move(ts)), values_(std::move(ys))
    {
        if (knots_.size() != values_.size() || knots_.size() < 2)
            throw std::invalid_argument("Need same number of times/values >= 2");
        if (!std::is_sorted(knots_.begin(), knots_.end()))
            throw std::invalid_argument("Times must be strictly increasing");
    }

    // Evaluate at time t
    T operator()(double t) const {
        if (t <= knots_.front()) return values_.front();
        if (t >= knots_.back())  return values_.back();

        // Fast path: increment cached index if query moved forward
        while (last_idx_ + 1 < knots_.size() && t > knots_[last_idx_+1]) {
            ++last_idx_;
        }
        // If query moved backwards, reset cache and search again
        while (last_idx_ > 0 && t < knots_[last_idx_]) {
            --last_idx_;
        }

        double x0 = knots_[last_idx_];
        double x1 = knots_[last_idx_+1];
        double alpha = (t - x0) / (x1 - x0);
        return (1.0 - alpha) * values_[last_idx_] + alpha * values_[last_idx_+1];
    }

    // Piecewise-constant derivative
    T derivative(double t) const {
        if (t <= knots_.front()) return slope(0);
        if (t >= knots_.back())  return slope(knots_.size()-2);

        // Reuse the cached index logic
        operator()(t); // updates last_idx_ for this t
        return slope(last_idx_);
    }

private:
    T slope(std::size_t i) const {
        double dt = knots_[i+1] - knots_[i];
        return (values_[i+1] - values_[i]) / dt;
    }

    std::vector<double> knots_;
    std::vector<T> values_;
    mutable std::size_t last_idx_ = 0; // cache, mutable for const operator()
};


//#################################################################################
// Save as CSV
template<typename Derived>
void save_csv(const std::string& filename, const Eigen::MatrixBase<Derived>& mat) {
    std::ofstream out(filename);
    out << std::setprecision(10);
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            out << mat(i,j);
            if (j + 1 < mat.cols()) out << ",";
        }
        out << "\n";
    }
}

// Load from CSV
Eigen::MatrixXd load_csv(const std::string& filename, int rows, int cols) {
    std::ifstream in(filename);
    Eigen::MatrixXd mat(rows, cols);
    for (int i = 0; i < rows; ++i) {
        std::string line;
        std::getline(in, line);
        std::stringstream ss(line);
        std::string cell;
        for (int j = 0; j < cols; ++j) {
            std::getline(ss, cell, ',');
            mat(i,j) = std::stod(cell);
        }
    }
    return mat;
}

//#################################################################################
using namespace MyUtils;
using Vec7 = Eigen::Matrix<double,7,1>;
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using namespace drake; // for all eigen types

int main(int argc, char** argv) 
{

    // std::string file_path = "/home/rosdrake/src/free_flight_test_qvu_2nd_lowSolverPrecision.npy";
    // std::string file_path = "/home/rosdrake/src/test_N60_hlow0.07/dtheta2.0/traj.npy";
    // std::string file_path = "/home/rosdrake/src/test_N60_hlow0.04/dtheta2.0/traj.npy";
    std::string file_path = "/home/rosdrake/src/test_N60_hlow0.03/dtheta1.0/traj.npy";
    cnpy::NpyArray arr = cnpy::npy_load(file_path);
    double* data = arr.data<double>();

    //print shape
    std::vector<unsigned long> shape = arr.shape;

    size_t nRow = 0;
    size_t nCol = 0;
    if (shape.size() == 1) 
    {
      nRow = 1;
      nCol = shape[0];
    } else if (shape.size() == 2){
      nRow = shape[0];
      nCol = shape[1];
    } else {
      std::cerr << "Unsupported shape with dim > 2" << std::endl;
    }

    //map into eigen VectorXd 
    Eigen::VectorXd sol_data = Eigen::Map<Eigen::VectorXd>(data, nRow*nCol);
    int N = 60;

    /* for free motion */
    // std::vector<std::string> var_names = {"q", "v", "u", "h"};
    // std::vector<int> dims = {7, 7, 7, 1};
    // std::vector<int> times = {N, N, N, N-1};
    // ProcessedTrajMapToMat sol_in_map;
    // sol_in_map = PreprocessSolutionToMat(sol_data, var_names, dims, times);
    // std::cout << "sol_in_map.q.rows(): " << sol_in_map["q"].rows() << std::endl;
    // std::cout << "sol_in_map.q.cols(): " << sol_in_map["q"].cols() << std::endl;
    // std::cout << "sol_in_map.h.rows(): " << sol_in_map["h"].rows() << std::endl;
    // std::cout << "sol_in_map.h.cols(): " << sol_in_map["h"].cols() << std::endl;

    /* for contact */
    std::vector<std::string> var_names = {"q", "v", "u", "ln", "lt", "v1", "w1", "ds", "dtheta", "h"};
    std::vector<int> dims = {7, 7, 7, 1, 2, 2, 1, 1, 1, 1};
    std::vector<int> times = {N, N, N, 1, 1, 1, 1, 1, 1, N-1};
    ProcessedTrajMapToMat sol_in_map;
    sol_in_map = PreprocessSolutionToMat(sol_data, var_names, dims, times);
    std::cout << "sol_in_map.q.rows(): " << sol_in_map["q"].rows() << std::endl;
    std::cout << "sol_in_map.q.cols(): " << sol_in_map["q"].cols() << std::endl;
    std::cout << "sol_in_map.h.rows(): " << sol_in_map["h"].rows() << std::endl;
    std::cout << "sol_in_map.h.cols(): " << sol_in_map["h"].cols() << std::endl;
    std::cout << "sol_in_map.ln.rows(): " << sol_in_map["ln"].rows() << std::endl;
    std::cout << "sol_in_map.ln.cols(): " << sol_in_map["ln"].cols() << std::endl;
    std::cout << "sol_in_map.lt.rows(): " << sol_in_map["lt"].rows() << std::endl;
    std::cout << "sol_in_map.lt.cols(): " << sol_in_map["lt"].cols() << std::endl;
    std::cout << "sol_in_map.v1.rows(): " << sol_in_map["v1"].rows() << std::endl;
    std::cout << "sol_in_map.v1.cols(): " << sol_in_map["v1"].cols() << std::endl;
    std::cout << "sol_in_map.w1.rows(): " << sol_in_map["w1"].rows() << std::endl;
    std::cout << "sol_in_map.w1.cols(): " << sol_in_map["w1"].cols() << std::endl;
    std::cout << "sol_in_map.ds.rows(): " << sol_in_map["ds"].rows() << std::endl;
    std::cout << "sol_in_map.ds.cols(): " << sol_in_map["ds"].cols() << std::endl;
    std::cout << "sol_in_map.dtheta.rows(): " << sol_in_map["dtheta"].rows() << std::endl;
    std::cout << "sol_in_map.dtheta.cols(): " << sol_in_map["dtheta"].cols() << std::endl;

    // build the plant 
    double h_sim = 0.001;
    std::string panda_file {"/home/rosdrake/src/drake_models/franka_description/urdf/panda_arm.urdf"};
    RigidTransform<double> X_W_base = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(0., 0., -90.) * M_PI / 180.),
                                                            Vector3<double>(0., -0.2, 0.));
    auto plant_ptr = std::make_unique<MultibodyPlant<double>>(h_sim);
    Parser parser(plant_ptr.get());
    parser.AddModels(panda_file);
    const auto& arm_base_frame = plant_ptr->GetFrameByName("panda_link0");
    plant_ptr->WeldFrames(plant_ptr->world_frame(), arm_base_frame, X_W_base);
    plant_ptr->Finalize();
    auto context_ptr = plant_ptr->CreateDefaultContext();

    // remove G from the feedforward u_ff
    Eigen::MatrixXd u_noG(sol_in_map["u"].rows(), sol_in_map["u"].cols());
    for (int i = 0; i < sol_in_map["u"].rows(); i++)
    {
        Eigen::VectorXd qi = sol_in_map["q"].row(i);
        //compute G
        plant_ptr->SetPositions(context_ptr.get(), qi);
        Eigen::VectorXd ui = sol_in_map["u"].row(i);
        Eigen::VectorXd Gi = plant_ptr->CalcGravityGeneralizedForces(*context_ptr);

        if (i == 0)
        {
            std::cout << "Gi: " << Gi.transpose() << std::endl;
            Eigen::VectorXd ui_noG = ui - Gi;
        }
        // std::cout << "Gi: " << Gi.transpose() << std::endl;
        // std::cout << "Gi.rows(): " << Gi.rows() << std::endl;
        // std::cout << "Gi.cols(): " << Gi.cols() << std::endl;
        else 
        {
            Eigen::VectorXd ui_noG = ui + Gi;
            u_noG.row(i) = ui_noG;
        }
    }

    std::cout << "u_noG.rows(): " << u_noG.rows() << std::endl;
    std::cout << "u_noG.cols(): " << u_noG.cols() << std::endl;
    std::cout << "u_noG: \n" << u_noG << std::endl;

    // save-to-csv
    std::string path_q = "/home/rosdrake/catkin_ws/src/linearmpc_panda/test3_N60_Euler_hlow0.03_dtheta1.0_q.csv";
    std::string path_v = "/home/rosdrake/catkin_ws/src/linearmpc_panda/test3_N60_Euler_hlow0.03_dtheta1.0_v.csv";
    std::string path_u = "/home/rosdrake/catkin_ws/src/linearmpc_panda/test3_N60_Euler_hlow0.03_dtheta1.0_u.csv";
    std::string path_h = "/home/rosdrake/catkin_ws/src/linearmpc_panda/test3_N60_Euler_hlow0.03_dtheta1.0_h.csv";
    save_csv(path_q, sol_in_map["q"]);
    save_csv(path_v, sol_in_map["v"]);
    save_csv(path_u, u_noG);
    save_csv(path_h, sol_in_map["h"]);

    auto loaded_q = load_csv(path_q, sol_in_map["q"].rows(), sol_in_map["q"].cols());
    std::cout << "loaded_q.rows(): " << loaded_q.rows() << std::endl;
    std::cout << "loaded_q.cols(): " << loaded_q.cols() << std::endl;
    //std::cout << "loaded_q: " << loaded_q << std::endl;

    auto loaded_v = load_csv(path_v, sol_in_map["v"].rows(), sol_in_map["v"].cols());
    std::cout << "loaded_v.rows(): " << loaded_v.rows() << std::endl;
    std::cout << "loaded_v.cols(): " << loaded_v.cols() << std::endl;
    //std::cout << "loaded_v: " << loaded_v << std::endl;

    auto loaded_u = load_csv(path_u, sol_in_map["u"].rows(), sol_in_map["u"].cols());
    std::cout << "loaded_u.rows(): " << loaded_u.rows() << std::endl;
    std::cout << "loaded_u.cols(): " << loaded_u.cols() << std::endl;
    //std::cout << "loaded_u: " << loaded_u << std::endl;

    //std::cout << "loaded_q: " << loaded_q << std::endl;
    auto loaded_h = load_csv(path_h, sol_in_map["h"].rows(), sol_in_map["h"].cols());
    std::cout << "loaded_h.rows(): " << loaded_h.rows() << std::endl;
    std::cout << "loaded_h.cols(): " << loaded_h.cols() << std::endl;

    // get times mapping from eigen vector to std::vector
    std::vector<double> ts;
    auto cumsum_h = cumulative_sum(loaded_h);
    std::cout << "cumsum_h: " << cumsum_h.transpose() << std::endl;
    ts.push_back(0.0); // start from 0 second
    for (int i = 0; i < cumsum_h.rows(); i++)
    {
        ts.push_back(cumsum_h(i));
    }

    // linear spline for q 
    std::vector<Vec7> qs;
    for (int i = 0; i < loaded_q.rows(); i++)
    {
        Vec7 q = loaded_q.row(i).transpose();
        qs.push_back(q);
    }
    LinearSpline<Vec7> q_spline(ts, qs);
    std::cout << "q_spline(0.0): " << q_spline(0.0).transpose() << std::endl;
    std::cout << "q_spline(0.12): " << q_spline(0.12).transpose() << std::endl;

    // linear spline for v 
    std::vector<Vec7> vs;
    for (int i = 0; i < loaded_v.rows(); i++)
    {
        Vec7 v = loaded_v.row(i).transpose();
        vs.push_back(v);
    }
    LinearSpline<Vec7> v_spline(ts, vs);
    std::cout << "v_spline(0.0): " << v_spline(0.0).transpose() << std::endl;
    std::cout << "v_spline(0.12): " << v_spline(0.12).transpose() << std::endl;
    
    // linear spline for u 
    std::vector<Vec7> us;
    for (int i = 0; i < loaded_u.rows(); i++)
    {
        Vec7 u = loaded_u.row(i).transpose();
        us.push_back(u);
    }
    LinearSpline<Vec7> u_spline(ts, us);
    std::cout << "u_spline(0.0): " << u_spline(0.0).transpose() << std::endl;
    std::cout << "u_spline(0.12): " << u_spline(0.12).transpose() << std::endl;

  return 0;
}