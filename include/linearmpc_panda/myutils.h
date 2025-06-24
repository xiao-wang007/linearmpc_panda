#pragma once 
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <stdexcept>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include "cnpy.h"

#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>


namespace MyUtils {
	//
	using drake::trajectories::PiecewisePolynomial;
	using drake::AutoDiffXd;
	using drake::multibody::MultibodyPlant;
	using drake::multibody::Parser;

	//
	using ProcessedTrajMapToMat = std::unordered_map<std::string, Eigen::MatrixXd>;

	//
	struct ProcessedSolution {
		ProcessedTrajMapToMat trajs;
		PiecewisePolynomial<double> x_ref_spline;
		PiecewisePolynomial<double> u_ref_spline; 

		//default constructor
		//ProcessedSolution() = default;
		ProcessedSolution()
        : trajs(), x_ref_spline(PiecewisePolynomial<double>()), u_ref_spline(PiecewisePolynomial<double>()) {}

		//const data members must be initialized via a constructor initializer list
		ProcessedSolution(ProcessedTrajMapToMat& in1,
						  PiecewisePolynomial<double>& in2,
						  PiecewisePolynomial<double>& in3)
			: trajs(in1), x_ref_spline(in2), u_ref_spline(in3) {}
		
		/* If ProcessedTrajMapToMat trajs has to be const, then a custom assign operator 
		   is needed, then do this: 
		//ProcessedTrajMapToMat trajs;
		ProcessedSolution& operator=(const ProcessedSolution& other) 
		{
			if (this != &other) 
			{ 
			  //since trajs is const, it cannot be reassigned.
			  //you can only copy other non-const members here.
			}

			return *this;
		}
		*/
		
		/* To allow reassignment, store trajs as a pointer or a reference 
		std::shared_ptr<const ProcessedTrajMapToMat> trajs;
		*/
	};

	//
	ProcessedTrajMapToMat PreprocessSolutionToMat(const Eigen::VectorXd& sol,
							   					  const std::vector<std::string>& var_names, 
							   					  const std::vector<int> dims,
							   					  const std::vector<int> nTimes);

    //
	template <typename T>
	std::vector<T> SlicingVector(const std::vector<T>& v,
			  					 int start,
			  					 int end);

	//
	template <typename Derived>
	Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> CumSum(const Eigen::MatrixBase<Derived>& vec);

	//
	template <typename T>
	void PrintStdVector(const std::vector<T>& vec);

	//
	const ProcessedSolution ProcessSolTraj(const std::string& file_path,
										   const std::vector<std::string>& var_names,
										   const std::vector<int> dims,
										   const std::vector<int> times);

	//
	void VisualizeMatSparsity(const Eigen::MatrixXd& mat);

	//
	void BuildArmOnlyPlant(std::unique_ptr<MultibodyPlant<double>>& plant_ptr);
}

