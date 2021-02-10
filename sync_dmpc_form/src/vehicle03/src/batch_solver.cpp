#include "batch_solver.h"

void BatchSolver::set_initial_states(const double& x,const double& y,const double& theta){

	xinit_ = x;
	yinit_ = y;
	thetainit_ = theta;
};
void BatchSolver::set_ref_states(const double& x,const double& y,const double& theta){
	xr_ = x;
	yr_ = y;
	thetar_ = theta;
};
void BatchSolver::set_obst_(const std::vector<std::vector<double>>& obst){
	obst_ = obst;
	p_ = obst_.size();
};
void BatchSolver::set_neighbors(const std::vector<std::vector<std::vector<double>>>& neig,std::mutex& neig_mtx){
	std::lock_guard<std::mutex> lk(neig_mtx);	
	neig_ = neig;
	m_ = neig.size();

};

void BatchSolver::Solve(std::vector<std::vector<double>>& pre_states,std::vector<std::vector<double>>& pre_inputs,bool& solve_success){

    typedef CPPAD_TESTVECTOR( double ) Dvector;

    size_t num_states =5*(N_-1)+10 + (4*(N_-1)+8)*(m_ + p_) + (m_-1)*(N_+1) + N_ + 1 ;
 //   std::cout<<"dsf"<<N_<<" "<<m_<<" "<<p_<<std::endl;
    Dvector xi(num_states);
    if(warmstart_.empty()){//
    		for(size_t i = 0; i < num_states; i++){
 			xi[i] = 0.0;

   		 }
	}else{
    		for(size_t i = 0; i < num_states; i++){
 			xi[i] = warmstart_[i];
   		 }
	}

    //std::cout<<xi<<std::endl;
    // lower and upper limits for x
    Dvector xl(num_states), xu(num_states);

	for(size_t j = 0; j < N_ + 1; j++){
		xl[j*5] = -10.0;xu[j*5] = 10.0;
		xl[j*5 + 1] = -10.0;xu[j*5 + 1] = 10.0;
		xl[j*5 + 2] = -4.0*3.14;xu[j*5 + 2] = 4.0*3.14;
		xl[j*5 + 3] = 0.0;xu[j*5 + 3] = 3.0;
		xl[j*5 + 4] = 0.0;xu[j*5 + 4] = 3.0;
	}


    size_t start_i = 5*(N_-1)+10; 
    for(size_t i = 0; i < m_;i++){
		for(size_t k = 0; k < N_+1;k++){
			xl[start_i + i*4*(N_+1) +k*4] = -1.0e19;xu[start_i + i*4*(N_+1)  + k*4] = 1.0e19;
			xl[start_i  + i*4*(N_+1) + k*4 + 1] = -1.0e19;xu[start_i  + i*4*(N_+1) + k*4 + 1] = 1.0e19;
			xl[start_i + i*4*(N_+1)  + k*4 + 2] = -1.0e19;xu[start_i + i*4*(N_+1)  + k*4 + 2] = 1.0e19;
			xl[start_i + i*4*(N_+1)  + k*4 + 3] = 0.0;xu[start_i + i*4*(N_+1)  + k*4 + 3] = 1.0e19;
		}
		
	}

	size_t start_obst = start_i + (4*(N_-1)+8)*m_;

	for(size_t j = 0; j < p_;j++){
		for(size_t k = 0; k < N_+1;k++){
			xl[start_obst  + j*4*(N_+1) + k*4] = -1.0e19;xu[start_obst  + j*4*(N_+1) + k*4] = 1.0e19;
			xl[start_obst  + j*4*(N_+1)+ k*4 + 1]= -1.0e19;xu[start_obst + j*4*(N_+1) + k*4+1]= 1.0e19;
			xl[start_obst  + j*4*(N_+1)+ k*4 + 2]= -1.0e19;xu[start_obst  + j*4*(N_+1) + k*4+2]= 1.0e19;
			xl[start_obst  + j*4*(N_+1)+ k*4 + 3]= 0.0;xu[start_obst  + j*4*(N_+1) + k*4+3]= 1.0e19;
		}
	}
	size_t start_form = start_obst + (4*(N_-1)+8)*p_;
    for(size_t i = 0; i < m_;i++){
		for(size_t k = 0; k < N_+1;k++){
			xl[start_form + i*(N_+1) +k] = -1.0e19;xu[start_form + i*(N_+1)  + k] = 1.0e19;
		
		}
		
	}	
	  


   // std::cout<<xl<<std::endl;
  //  std::cout<<xu<<std::endl;
    // lower and upper limits for g
    size_t size_tmp = (m_ + p_ -1)*(3*N_+3) + 6*N_ + 7 + (m_ - 1)*(N_+1) + N_ ;
 
    Dvector gl(size_tmp), gu(size_tmp);
    for(size_t i = 0; i < 3*N_ ; i++){// kinematics
	    gl[i] = 0.0;     gu[i] = 0.0;
    }



   for(size_t i = 3*N_  ; i < 3*N_+m_*(3*N_+3); i++){// inter-vehicles  avoidance
	    gl[i] = 0.0;     gu[i] = 1.0e19;
	}


	 
   for(size_t i = 3*N_+m_*(3*N_+3) ; i <  3*N_+m_*(3*N_+3) + 3; i++){// intial constraints
	    gl[i] = 0.0;     gu[i] = 0.0;
	}

   for(size_t i = 3*N_+m_*(3*N_+3) + 3; i <(m_ + p_ -1)*(3*N_+3) + 6*N_ + 6; i++){// obst-vehicles constraints
	    gl[i] = 0.0;     gu[i] = 1.0e19;
	}
   for(size_t i = (m_ + p_ -1)*(3*N_+3) + 6*N_ + 6; i <(m_ + p_ -1)*(3*N_+3) + 6*N_ + 7 + (m_ - 1)*(N_+1) + N_; i++){// formation constraints
	    gl[i] = 0.0;     gu[i] = 0.0;
	}


//std::cout<<3*N_*(m_+(m_+1)*(m_-2)/2+1)+3<<"jijijkjk "<<std::endl;
  // std::cout<<gl<<std::endl;
  //std::cout<<gu<<std::endl;
    // object that computes objective and constraints
 //exit(0);


    // N_, m_, xr_, yr_, thetar_, d_, xinit_,yinit_, thetainit_, ts_, safety_dist_
    FG_eval fg_eval(N_, m_,p_,xr_, yr_, thetar_, d_, xinit_,yinit_, thetainit_, ts_, safety_dist_,obst_,neig_);

    // options
    std::string options;
    // turn off any printing
    options += "Integer print_level  0\n";
    //options += "Retape  true\n";
    options += "Sparse true        forward\n";
    options += "Sparse true         reverse\n";
   // options += "Numeric max_cpu_time          0.5\n";
 

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    auto start_t = std::chrono::system_clock::now();
    
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution
    );

    auto end   = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_t);
     std::cout <<  "花费了" 
     << double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den 
     << "秒" << std::endl;
    //
    // Check some of the solution values
    //
    std::cout<<"state : "<<solution.status<<std::endl;
    std::cout<<"obj : "<<solution.obj_value<<std::endl;
    //
    //exit(0);
    bool ok = true;
    ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success);//||(solution.status == CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point);


    if(ok){

	    for(size_t j = 0; j < N_+1;j++){
			pre_states[j][0] = solution.x[j*5];
			pre_states[j][1] = solution.x[j*5 + 1];
			pre_states[j][2] = solution.x[j*5 + 2];
			pre_inputs[j][0] = solution.x[j*5 + 3];
			pre_inputs[j][1] = solution.x[j*5 + 4];
				//std::cout<<" ssss: "<<pre_states[i][j][0]<<" "<<pre_states[i][j][1]<<" "<<pre_states[i][j][2]<<std::endl;
		}

		
	    warmstart_.clear();
	    for(size_t i = 0; i<5*(N_-1)+10 + (4*(N_-1)+8)*(m_ + p_) + (m_-1)*(N_+1) + N_ + 1;i++){
			
			warmstart_.push_back(solution.x[i]);

		}
	
	}
 

    


    solve_success = ok;




    return;


}
