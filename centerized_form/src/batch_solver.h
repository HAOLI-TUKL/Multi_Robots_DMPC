#ifndef _BATCHSOLVER_H
#define _BATCHSOLVER_H
#include "fg_eval.h"


# include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
class BatchSolver{
	public:
	BatchSolver(size_t N_,size_t m_,std::vector<double> xr_,std::vector<double> yr_,std::vector<double> thetar_,double d_,std::vector<double> xinit_,std::vector<double> yinit_,std::vector<double> thetainit_,double ts_,double safety_dist_,std::vector<std::vector<double>> obst_)
	:N_(N_),m_(m_),xr_(xr_),yr_(yr_),thetar_(thetar_),d_(d_),xinit_(xinit_),yinit_(yinit_),thetainit_(thetainit_),ts_(ts_),safety_dist_(safety_dist_),obst_(obst_){

	};
	bool Solve(std::vector<std::vector<std::vector<double>>>& pre_states,std::vector<std::vector<std::vector<double>>>& pre_inputs);
	void set_initial_states(const std::vector<double>& x,const std::vector<double>& y,const std::vector<double>& theta);
	void set_ref_states(const std::vector<double>& x,const std::vector<double>& y,const std::vector<double>& theta);
	void set_obst_(const std::vector<std::vector<double>>& obst);
	private:
	size_t N_;
	size_t m_;
	std::vector<double> xr_;
	std::vector<double> yr_;
	std::vector<double> thetar_;
	std::vector<double> xinit_;
	std::vector<double> yinit_;
	std::vector<double> thetainit_;
	std::vector<double> warmstart_;
	std::vector<std::vector<double>> obst_;
	double ts_ ;
	double d_;
	double safety_dist_;
};

#endif
