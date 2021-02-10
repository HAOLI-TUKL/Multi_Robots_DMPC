#ifndef _BATCHSOLVER_H
#define _BATCHSOLVER_H
#include "fg_eval.h"
#include <atomic>
#include <mutex>
# include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>

class BatchSolver{
	public:
	BatchSolver(size_t N_,double xr_,double yr_,double thetar_,double d_,double xinit_,double yinit_,double thetainit_,double ts_,double safety_dist_,std::vector<std::vector<double>> obst_,std::vector<std::vector<std::vector<double>>> neig_)
	:N_(N_),xr_(xr_),yr_(yr_),thetar_(thetar_),d_(d_),xinit_(xinit_),yinit_(yinit_),thetainit_(thetainit_),ts_(ts_),safety_dist_(safety_dist_),obst_(obst_),neig_(neig_){
		m_ = neig_.size();
		p_ = obst_.size();
	};
	void Solve(std::vector<std::vector<double>>& pre_states,std::vector<std::vector<double>>& pre_inputs,bool& solve_success);
	void set_initial_states(const double& x,const double& y,const double& theta);
	void set_neighbors(const std::vector<std::vector<std::vector<double>>>& neig,std::mutex& neig_mtx);
	void set_ref_states(const double& x,const double& y,const double& theta);
	void set_obst_(const std::vector<std::vector<double>>& obst);
	private:
	size_t N_;
	size_t m_;
	size_t p_;
	double xr_;
	double yr_;
	double thetar_;
	double xinit_;
	double yinit_;
	double thetainit_;
	std::vector<std::vector<double>> obst_;
	std::vector<std::vector<std::vector<double>>> neig_;
	std::vector<double> warmstart_;
	double ts_ ;
	double d_;
	double safety_dist_;


};

#endif
