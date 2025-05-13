#include "myutils.h"

namespace MyUtils{

//####################################################################################################
 ProcessedTrajMapToMat PreprocessSolutionToMat(const Eigen::VectorXd& sol,
											   const std::vector<std::string>& var_names, 
											   const std::vector<int> dims,
											   const std::vector<int> nTimes) 
 {  // 
	ProcessedTrajMapToMat trajs;

	// slice the vector
	int index = 0;
	int dim = 0;
	int nt = 0;
	Eigen::VectorXd temp;

	size_t nEntries = var_names.size();
	//std::cout << "dim of sol: " << sol.size() << std::endl;
	for (int i = 0; i < nEntries; i++)
	{
	  dim = dims.at(i);
	  nt = nTimes.at(i);
	  temp = sol.segment(index, (dim*nt)); // 2nd arg is the length not end point 
	  index += (dim*nt);

	  // Map only return a view, not the data
	  trajs[var_names[i]] = Eigen::MatrixXd(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 
	  										Eigen::Dynamic, Eigen::ColMajor>>(temp.data(), nt, dim) ); 
	}
	return trajs;
}


//####################################################################################################
template <typename T>
std::vector<T> SlicingVector(const std::vector<T>& v,
							 int start,
							 int end) {
	if (end > v.size()) end = v.size();
	return std::vector<T>(v.begin() + start, v.begin() + end);
}


//####################################################################################################
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> 
CumSum(const Eigen::MatrixBase<Derived>& vec_in) 
{
	Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> vec_out(vec_in.size());
	if (vec_in.size() > 0) {
		vec_out(0) = vec_in(0);
		for (int i = 1; i < vec_in.size(); ++i) {
			vec_out(i) = vec_out(i-1) + vec_in(i);
			}
		}
	return vec_out;
}


//####################################################################################################
template <typename T>
void PrintStdVector(const std::vector<T>& vec)
{
	for (const auto& ele : vec) {
		std::cout << ele << std::endl;
		std::cout << std::endl;
	}
}


//####################################################################################################
const ProcessedSolution ProcessSolTraj(const std::string& file_path,
											      			     const std::vector<std::string>& var_names,
											      			     const std::vector<int> dims,
											      			     const std::vector<int> times)
{
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
    Eigen::VectorXd data_vec = Eigen::Map<Eigen::VectorXd>(data, nRow*nCol);

    //process the solution trajectory
    ProcessedTrajMapToMat trajs = PreprocessSolutionToMat(data_vec, var_names, dims, times);

    ////print the map for checking
    //for (const auto& pair: trajs){
      //std::cout << pair.first << " : \n" << pair.second << std::endl; 
    //}

    int nIntervals = trajs["h"].size();
    Eigen::VectorXd ts(nIntervals + 1);
    ts(0) = 0.;
    ts.tail(nIntervals) = CumSum(trajs["h"]);
	Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1> ts_ad = ts.cast<AutoDiffXd>();

    //hstack q&v into x
    int N = times[0];
    Eigen::MatrixXd x(N, trajs["q_panda"].cols() + trajs["v_panda"].cols());
    x << trajs["q_panda"], trajs["v_panda"];

	//Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, Eigen::Dynamic> x_ad = x.cast<AutoDiffXd>();
	//Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, Eigen::Dynamic> u_ad = trajs["us"].cast<AutoDiffXd>();

    //PrintStdVector(x_ref);
    auto x_ref_spline = PiecewisePolynomial<double>::FirstOrderHold(ts, x.transpose());
    auto u_ref_spline = PiecewisePolynomial<double>::FirstOrderHold(ts, trajs["us"].transpose());
    //const auto x_ref_spline_ad = PiecewisePolynomial<AutoDiffXd>::FirstOrderHold(ts_ad, x_ad.transpose());
    //const auto u_ref_spline_ad = PiecewisePolynomial<AutoDiffXd>::FirstOrderHold(ts_ad, u_ad.transpose());

    auto psol = ProcessedSolution(trajs, x_ref_spline, u_ref_spline);

    return psol;
}

//####################################################################################################
void VisualizeMatSparsity(const Eigen::MatrixXd& mat)
{
	// Print sparsity pattern
	for (int i = 0; i < mat.rows(); ++i) 
	{
		for (int j = 0; j < mat.cols(); ++j)
		{
			if (mat.coeff(i, j) != 0.0)
				std::cout << "X ";
			else
				std::cout << ". ";
		}
		std::cout << std::endl;
	}
}










}
