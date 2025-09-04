
#include <vector>
#include <string>
#include <Eigen/Dense>
//#include "../include/linearmpc_panda/myutils.h"
#include <linearmpc_panda/myutils.h>
#include <fstream>

#include <Eigen/Dense>
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

int main(int argc, char** argv) 
{

    std::string file_path = "/home/rosdrake/catkin_ws/src/linearmpc_panda/free_flight_test_qvu.npy";
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
    std::vector<std::string> var_names = {"q", "v", "u", "h"};
    std::vector<int> dims = {7, 7, 7, 1};
    std::vector<int> times = {N, N, N, N-1};
    ProcessedTrajMapToMat sol_in_map;
    sol_in_map = PreprocessSolutionToMat(sol_data, var_names, dims, times);
    std::cout << "sol_in_map.q.rows(): " << sol_in_map["q"].rows() << std::endl;
    std::cout << "sol_in_map.q.cols(): " << sol_in_map["q"].cols() << std::endl;
    std::cout << "sol_in_map.h.rows(): " << sol_in_map["h"].rows() << std::endl;
    std::cout << "sol_in_map.h.cols(): " << sol_in_map["h"].cols() << std::endl;

    // save-to-csv
    save_csv("/home/rosdrake/catkin_ws/src/linearmpc_panda/free_flight_test_q.csv", sol_in_map["q"]);
    save_csv("/home/rosdrake/catkin_ws/src/linearmpc_panda/free_flight_test_v.csv", sol_in_map["v"]);
    save_csv("/home/rosdrake/catkin_ws/src/linearmpc_panda/free_flight_test_u.csv", sol_in_map["u"]);
    save_csv("/home/rosdrake/catkin_ws/src/linearmpc_panda/free_flight_test_h.csv", sol_in_map["h"]);

    auto loaded_q = load_csv("/home/rosdrake/catkin_ws/src/linearmpc_panda/free_flight_test_q.csv", sol_in_map["q"].rows(), sol_in_map["q"].cols());
    std::cout << "loaded_q.rows(): " << loaded_q.rows() << std::endl;
    std::cout << "loaded_q.cols(): " << loaded_q.cols() << std::endl;
    //std::cout << "loaded_q: " << loaded_q << std::endl;

    // get times mapping from eigen vector to std::vector
    std::vector<double> ts;
    auto cumsum_h = cumulative_sum(sol_in_map["h"]);
    std::cout << "cumsum_h: " << cumsum_h.transpose() << std::endl;
    ts.push_back(0.0); // start from 0 second
    for (int i = 0; i < cumsum_h.rows(); i++)
    {
        ts.push_back(cumsum_h(i));
    }

    // linear spline for q 
    std::vector<Vec7> qs;
    for (int i = 0; i < sol_in_map["q"].rows(); i++)
    {
        Vec7 q = sol_in_map["q"].row(i).transpose();
        qs.push_back(q);
    }
    LinearSpline<Vec7> q_spline(ts, qs);
    std::cout << "q_spline(0.0): " << q_spline(0.0).transpose() << std::endl;
    std::cout << "q_spline(0.12): " << q_spline(0.12).transpose() << std::endl;

    // linear spline for v 
    std::vector<Vec7> vs;
    for (int i = 0; i < sol_in_map["v"].rows(); i++)
    {
        Vec7 v = sol_in_map["v"].row(i).transpose();
        vs.push_back(v);
    }
    LinearSpline<Vec7> v_spline(ts, vs);
    std::cout << "v_spline(0.0): " << v_spline(0.0).transpose() << std::endl;
    std::cout << "v_spline(0.12): " << v_spline(0.12).transpose() << std::endl;
    
    // linear spline for u 
    std::vector<Vec7> us;
    for (int i = 0; i < sol_in_map["u"].rows(); i++)
    {
        Vec7 u = sol_in_map["u"].row(i).transpose();
        us.push_back(u);
    }
    LinearSpline<Vec7> u_spline(ts, us);
    std::cout << "u_spline(0.0): " << u_spline(0.0).transpose() << std::endl;
    std::cout << "u_spline(0.12): " << u_spline(0.12).transpose() << std::endl;

  return 0;
}