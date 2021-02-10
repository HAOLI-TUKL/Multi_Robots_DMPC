#ifndef _FG_EVAL_H
#define _FG_EVAL_H

#include <vector>
#include <cppad/ipopt/solve.hpp>

//0 29;30 41;42 89
//vehicles;inter-vehicles;obst-vehicles
//0;  1 12;          13 21;     22 27;     28 63
//obj;kinematics(12);inter-v(9);initial(6);obst-v(36)


namespace {
    using CppAD::AD;

    class FG_eval {
    private:
	size_t N_;
	size_t m_;// num of neighbors
	size_t p_;// num of obsts
	double xr_;
	double yr_;
	double thetar_;
	double xinit_;
	double yinit_;
	double thetainit_;
	std::vector<std::vector<double>> obst_;
	std::vector<std::vector<std::vector<double>>> neig_;
	double ts_ ;
	double d_;
	double safety_dist_;
    public:
	FG_eval(size_t N_,size_t m_,size_t p_,double xr_,double yr_,double thetar_,double d_,double xinit_,double yinit_,double thetainit_,double ts_,double safety_dist_,std::vector<std::vector<double>> obst_,std::vector<std::vector<std::vector<double>>> neig_)
	:N_(N_),m_(m_),p_(p_),xr_(xr_),yr_(yr_),thetar_(thetar_),d_(d_),xinit_(xinit_),yinit_(yinit_),thetainit_(thetainit_),ts_(ts_),safety_dist_(safety_dist_),obst_(obst_),neig_(neig_){

	};
        typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;


        void operator()(ADvector& fg, const ADvector& x)
        {   assert( fg.size() ==  (m_ + p_ -1)*(3*N_+3) + 6*N_ + 7 + (m_ - 1)*(N_+1) + N_ + 1);
            assert( x.size()  == 5*(N_-1)+10 + (4*(N_-1)+8)*(m_ + p_) + (m_-1)*(N_+1) + N_ + 1  );

/**	m vehicles; N horizon; N+1 groups states;N+1 groups input; overall vars: 5(N-1)+10 + (4(N-1)+8)(m + p) + (m-1)(N+1) + N + 1
*	========= x y z linearvelocity angle
*	vehicle 1 : (x0 x1 x2 x3 x4)(x5 x6 x7 x8 x9)...(x 5(N-1)+5,x 5(N-1)+6,x 5(N-1)+7,5(N-1)+8,5(N-1)+9) start_index: 0 ;end_index:5(N-1)+9;
*
*	========= plane_a0 plane_a1 plane_b soft_factor (inter-vehicles seperation planes)
*	------ set var1 = 5(N-1)+10
*	vehicle/neighbor 11 : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7 ) start_index: var1;end_index:var1+4(N-1)+7
*	vehicle/neighbor 12 : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7) start_index: var1 + 4(N-1)+8;end_index:var1 + 4(N-1)+8+ 4(N-1)+7
*	...
* 	vehicle/neighbor 1m : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7) start_index: var1 + (4(N-1)+8)(m-1);end_index:var1 + (4(N-1)+8)(m-1)+ 4(N-1)+7
*	
*	========== vehicle-obstacles seperation planes
*	------ set start = 5(N-1)+10 + (4(N-1)+8)m
*	vehicle 11 : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7 ) start_index = start; end_index = start + 4(N-1)+7 ;
*	vehicle 12 : start_index = start + 4(N-1)+8; end_index = start+ 4(N-1)+8 + 4(N-1)+7;
*	vehicle 1p : start_index = start + (4(N-1)+8)(p-1); end_index = start+ (4(N-1)+8)(p-1)+ 4(N-1)+7;
*
*	========== soft factors for formation	
*       ------ set start =  5(N-1)+10 + (4(N-1)+8)(m + p)
*	vehicle 1: a0 a1 ... aN    start_index = start ; end_index = start + N
*	vehicle 2: start_index = start+ (N+1) ; end_index = start + (N + 1) + N
*	...
*	vehicle m: start_index = start+ (m-1)(N+1) ; end_index = start +  (m-1)(N+1) + N
**/

	
	std::vector<AD<double>> v_states;
	for(size_t j = 0; j < N_+1;j++){
		AD<double> x_tmp1 = x[j*5];
		v_states.push_back(x_tmp1);
		AD<double> x_tmp2 = x[j*5 + 1];
		v_states.push_back(x_tmp2);
		AD<double> x_tmp3 = x[j*5 + 2];
		v_states.push_back(x_tmp3);
		AD<double> x_tmp4 = x[j*5 + 3];
		v_states.push_back(x_tmp4);
		AD<double> x_tmp5 = x[j*5 + 4];
		v_states.push_back(x_tmp5);
	//std::cout<<j*5<<" "<<j*5+1<<" "<<j*5+2<<" "<<j*5+3<<" "<<j*5+4<<std::endl;
	}



	    // vehicle - neighbors
	std::vector<std::vector<AD<double>>> v_planes;
	size_t start_nei = 5*(N_-1)+10;
	size_t step = 4*(N_-1)+8;
	for(size_t i = 0; i < m_;i++){
		std::vector<AD<double>> vv;
		for(size_t k = 0; k < N_+1;k++){
			AD<double> x_tmp1 = x[start_nei + i*step + k*4];
			vv.push_back(x_tmp1);
			AD<double> x_tmp2 = x[start_nei + i*step + k*4 + 1];
			vv.push_back(x_tmp2);
			AD<double> x_tmp3 = x[start_nei + i*step + k*4 + 2];
			vv.push_back(x_tmp3);
			AD<double> x_tmp4= x[start_nei + i*step + k*4 + 3];
			vv.push_back(x_tmp4);
	//std::cout<<start_nei + i*step + k*4<<" "<<start_nei + i*step + k*4 + 1<<" "<<start_nei + i*step + k*4 + 2<<" "<<start_nei + i*step + k*4 + 3<<std::endl;
		}
		v_planes.push_back(vv);
	    }

	 std::vector<std::vector<AD<double>>> v_obst;
	 size_t start_obst = start_nei + (4*(N_-1)+8)*m_;
	 for(size_t j = 0; j < p_;j++){
		std::vector<AD<double>> ob;
		for(size_t k = 0; k < N_+1;k++){
			AD<double> x_tmp1 = x[start_obst + j*step + k*4];
			ob.push_back(x_tmp1);
			AD<double> x_tmp2 = x[start_obst + j*step + k*4 + 1];
			ob.push_back(x_tmp2);
			AD<double> x_tmp3 = x[start_obst + j*step + k*4 + 2];
			ob.push_back(x_tmp3);
			AD<double> x_tmp4= x[start_obst  + j*step + k*4 + 3];
			ob.push_back(x_tmp4);	
		//	std::cout<<start_obst +j*step + k*4<<" "<<start_obst + j*step + k*4 + 1<<" "<<start_obst + j*step + k*4 + 2<<" "<<start_obst + j*step + k*4 + 3<<std::endl;
		}
		v_obst.push_back(ob);
	};

	std::vector<std::vector<AD<double>>> v_form;
	size_t start_form = start_obst +  (4*(N_-1)+8)*p_;
	 for(size_t j = 0; j < m_;j++){
		std::vector<AD<double>> fo;
		for(size_t k = 0; k < N_+1;k++){
			AD<double> x_tmp1 = x[start_form + j*(N_+1) + k];
			fo.push_back(x_tmp1);	
		//	std::cout<<start_obst +j*step + k*4<<" "<<start_obst + j*step + k*4 + 1<<" "<<start_obst + j*step + k*4 + 2<<" "<<start_obst + j*step + k*4 + 3<<std::endl;
		}
		v_form.push_back(fo);
	};	


		    

/**
*	
*		
*	kinematics : 
*		fg[1] = x0 + ts*x3*cos(x2) - x5;
*		fg[2] = x1 + ts*x3*sin(x2) - x6 ;
*		fg[3] = x2 + ts*tan(x4)/d - x7;
*
*(1)	x1  to avoid obst 1 :
*		fg[7] = - (x3 * a0 + x4 * a1)  + a2 - safety + a3>= 0
*		fg[8] = (obstx1 * a0 + obsty1 * a1) - a2 - safety + a3>= 0
*		fg[9] =  - (a0*a0 + a1*a1) + 1.0 >= 0
*
*(2)	x2 to avoid obst 1 :
*		fg[10] = - (x6 * a4 + x7 * a5)  + a6 - safety + a7>= 0
*		fg[11] = (obstx1 * a4 + obsty1 * a5) - a6 - safety + a7>= 0
*		fg[12] = -(a4*a4 + a5*a5) + 1 >= 0
*
*
*	inital constraints
*	        fg[13] = x0 - xinit;
*	        fg[14] = x1 - yinit;
*		fg[15] = x2 - thetainit;  
**/

	// f(x)
	fg[0]=0;
        // reference
	    
	for(size_t j = 0; j < N_+1;j++){
		fg[0] += (v_states[j*5] - xr_)*(v_states[j*5] - xr_)+(v_states[j*5+1] - yr_)*(v_states[j*5+1] - yr_);
		fg[0] += v_states[j*5+3] *0.01*v_states[j*5+3] +v_states[j*5+4]*0.01*v_states[j*5+4];
		//std::cout<<"ss3 : "<<j*5<<" "<<j*5+1<<std::endl;
	}	
		

	// penalize soft factors of inter-vehicles
	for(size_t i = 0; i < m_ ; i++){
		for(size_t k = 0; k < N_+ 1;k++){
			//std::cout<<"ss4 : "<<k*4+3<<std::endl;
			fg[0] += (v_planes[i][k*4+3])*100000000*(v_planes[i][k*4+3]);
			//std::cout<<"ss4 : "<<k*4+3<<std::endl;
		}			
	}

	// penalize soft factors of obst-vehicles
	for(size_t j = 0; j < p_ ;j++){
		for(size_t k = 0; k < N_+ 1;k++){
			fg[0] += (v_obst[j][k*4+3])*100000000*(v_obst[j][k*4+3]);
		//	std::cout<<"ss11 : "<<k*4+3<<std::endl;
		}
	}	

	// penalize formation
	#if 1
	for(size_t i = 0; i < m_ ; i++){
		for(size_t k = 0; k < N_+ 1;k++){
			fg[0] += (v_form[i][k])*0.05*(v_form[i][k]);
			//std::cout<<"ss4 : "<<k*4+3<<std::endl;
		}			
	}	
	#endif
	




	    // kinematics constraints for differential drive:

	for(size_t j = 0; j < N_;j++){
		
			fg[j*3 + 1] = v_states[j*5] + 1.0/2.0 * (v_states[j*5+3]+v_states[j*5+4])*cos(v_states[j*5+2])*ts_ - v_states[j*5+5];
			fg[j*3 + 2] = v_states[j*5+1] + 1.0/2.0 * (v_states[j*5+3]+v_states[j*5+4])*sin(v_states[j*5+2])*ts_ - v_states[j*5+6];
			fg[j*3 + 3] = v_states[j*5+2] +1.0/d_*(v_states[j*5+4] - v_states[j*5+3])*ts_ - v_states[j*5+7];




#if 0
		//std::cout<<" vvvvvvvvvv    "<<v_states[j*5+3]<<"  "<<v_states[j*5+4]<<std::endl;	
		if(abs(v_states[j*5+4] - v_states[j*5+3]) < 0.0001 ){
			fg[j*3 + 1] = v_states[j*5] + v_states[j*5+3]*cos(v_states[j*5+2])*ts_ - v_states[j*5+5];
			fg[j*3 + 2] = v_states[j*5+1] + v_states[j*5+3]*sin(v_states[j*5+2])*ts_ - v_states[j*5+6];		
			fg[j*3 + 3] = v_states[j*5+2] - v_states[j*5+7];
			std::cout<<" xiangtong    "<<std::endl;	
		}else if(abs(v_states[j*5+4] + v_states[j*5+3]) < 0.0001   ){
			fg[j*3 + 1] = v_states[j*5]- v_states[j*5+5];
			fg[j*3 + 2] = v_states[j*5+1] - v_states[j*5+6];		
			fg[j*3 + 3] = v_states[j*5+2] + 2.0*v_states[j*5+4]*ts_/d_ - v_states[j*5+7];
			std::cout<<" xiangfang    "<<std::endl;	

		}else{

			/**AD<double> R = d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3]);
			AD<double> W = (v_states[j*5+4]-v_states[j*5+3]) / d_;
			AD<double> ICCx = v_states[j*5] - R * sin(v_states[j*5+2]);
			AD<double> ICCy = v_states[j*5+1] + R * cos(v_states[j*5+2]);
			//std::cout<<" R    "<<R<<std::endl;	
			//std::cout<<" W    "<<W<<std::endl;	
			//std::cout<<" ICCx    "<<ICCx<<std::endl;	
			//std::cout<<" ICCy    "<<ICCy<<std::endl;	
	
	
			fg[j*3 + 1] = cos(W * ts_)*(v_states[j*5] - ICCx) - sin(W * ts_) * (v_states[j*5+1] - ICCy) + ICCx - v_states[j*5+5];
			fg[j*3 + 2] = sin(W * ts_)*(v_states[j*5] - ICCx) + cos(W * ts_) * (v_states[j*5+1] - ICCy) + ICCy - v_states[j*5+6];
			fg[j*3 + 3] = v_states[j*5+2] + W* ts_ - v_states[j*5+7];**/
			//std::cout<<" 1    "<<fg[j*3 + 1]<<std::endl;	
			//std::cout<<" 2    "<<fg[j*3 + 2]<<std::endl;	
			//std::cout<<" 3    "<<fg[j*3 + 3]<<std::endl;
			


			/**AD<double> W = (v_states[j*5+4]-v_states[j*5+3]) / d_;
			AD<double> ICCx = v_states[j*5] -  (d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3]))* sin(v_states[j*5+2]);
			AD<double> ICCy = v_states[j*5+1] + (d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3])) * cos(v_states[j*5+2]);**/


			fg[j*3 + 1] = cos((v_states[j*5+4]-v_states[j*5+3]) / d_ * ts_)*(v_states[j*5] - (v_states[j*5] -  (d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3]))* sin(v_states[j*5+2]))) - sin((v_states[j*5+4]-v_states[j*5+3]) / d_ * ts_) * (v_states[j*5+1] - (v_states[j*5+1] + (d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3])) * cos(v_states[j*5+2]))) + (v_states[j*5] -  (d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3]))* sin(v_states[j*5+2])) - v_states[j*5+5];
			fg[j*3 + 2] = sin((v_states[j*5+4]-v_states[j*5+3]) / d_ * ts_)*(v_states[j*5] - (v_states[j*5] -  (d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3]))* sin(v_states[j*5+2]))) + cos((v_states[j*5+4]-v_states[j*5+3]) / d_ * ts_) * (v_states[j*5+1] - (v_states[j*5+1] + (d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3])) * cos(v_states[j*5+2]))) + (v_states[j*5+1] + (d_/2.0 * (v_states[j*5+3]+v_states[j*5+4])/(v_states[j*5+4]-v_states[j*5+3])) * cos(v_states[j*5+2])) - v_states[j*5+6];
			fg[j*3 + 3] = v_states[j*5+2] + (v_states[j*5+4]-v_states[j*5+3]) / d_* ts_ - v_states[j*5+7];

			std::cout<<" qita sssjsjsjdsd    "<<std::endl;	
		}
#endif 
				
	}; //fg[3N] is the last constraint 




		
	    // inter-vehicles avoidance constraints:

	    size_t start_avo = 3*N_+ 1;
	    for(size_t i = 0; i < m_;i++){
			for(size_t k = 0; k < N_+1;k++){
				fg[start_avo  + i*3*(N_+1) + k*3] = - (v_states[k*5] * v_planes[i][k*4] + v_states[k*5+1] * v_planes[i][k*4+1]) + v_planes[i][k*4+2] - safety_dist_ + v_planes[i][k*4+3];
				fg[start_avo  + i*3*(N_+1)  + k*3 + 1] =(neig_[i][k][0] * v_planes[i][k*4] + neig_[i][k][1] * v_planes[i][k*4+1]) - v_planes[i][k*4+2] - safety_dist_ + v_planes[i][k*4+3];
				fg[start_avo  + i*3*(N_+1)  + k*3 + 2] = -(v_planes[i][k*4] * v_planes[i][k*4] + v_planes[i][k*4+1] * v_planes[i][k*4+1]) +1.0;
		//std::cout<<"ss6 : "<<start_avo  + i*3*(N_+1) + k*3<<" "<<start_avo  + i*3*(N_+1) + k*3 +1<<" "<<start_avo  + i*3*(N_+1) + k*3+2<<std::endl;
			}
	        };//fg[ 3*N_+m_*(3*N_+3)] is the last constraint



	   


	    // inital constraints: 
	    
	    size_t start_init =  3*N_+m_*(3*N_+3) + 1;
	    fg[start_init] = v_states[0] - xinit_;
	    fg[start_init +1] = v_states[1] - yinit_;
	    fg[start_init +2] = v_states[2] - thetainit_;
	 //fg[ 3*N_+m_*(3*N_+3) + 3] is the last constraint
	//std::cout<<"ss7 : "<<start_init<<" "<<start_init +1<<" "<<start_init+2<<std::endl;


	    // obst-vehicles avoidance constraints
	    size_t start_obst_v =  3*N_+m_*(3*N_+3) + 4;
	//std::cout<<start_obst_v<<std::endl;
	    
		for(size_t j = 0; j < p_;j++){
			for(size_t k = 0; k < N_+1;k++){
				fg[start_obst_v + j*(N_+1)*3 + k*3] = - (v_states[k*5] * v_obst[j][k*4] + v_states[k*5+1] * v_obst[j][k*4+1]) + v_obst[j][k*4+2] - safety_dist_ + v_obst[j][k*4+3];
				fg[start_obst_v + j*(N_+1)*3 + k*3 + 1] =  (obst_[j][0] * v_obst[j][k*4] + obst_[j][1] * v_obst[j][k*4+1]) - v_obst[j][k*4+2] - safety_dist_ + v_obst[j][k*4+3];
				fg[start_obst_v + j*(N_+1)*3 + k*3 + 2] =  -(v_obst[j][k*4]*v_obst[j][k*4] + v_obst[j][k*4+1]* v_obst[j][k*4+1]) + 1.0 ;
		//std::cout<<"ss8 : "<<start_obst_v + j*(N_+1)*3 + k*3<<" "<<start_obst_v + j*(N_+1)*3 + k*3+1<<" "<<start_obst_v + j*(N_+1)*3 + k*3+2<<std::endl;
			}
		}//fg[ (m_ + p_ -1)*(3*N_+3) + 6*N_ + 6] is the last constraint
	  
	size_t start_form_v = (m_ + p_ -1)*(3*N_+3) + 6*N_ + 7;
		for(size_t j = 0; j < m_;j++){
			for(size_t k = 0; k < N_+1;k++){
				fg[start_form_v + j*(N_+1) + k] = (v_states[k*5]-neig_[j][k][0])*(v_states[k*5]-neig_[j][k][0])+(v_states[k*5+1]-neig_[j][k][1])*(v_states[k*5+1]-neig_[j][k][1])-4.0+v_form[j][k];
			}		
		}//fg[ (m_ + p_ -1)*(3*N_+3) + 6*N_ + 7 + (m_ - 1)*(N_+1) + N_] 
		

            return;
        }
    };
}

#endif 
