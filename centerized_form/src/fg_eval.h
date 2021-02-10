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
	size_t m_;
	std::vector<double> xr_;
	std::vector<double> yr_;
	std::vector<double> thetar_;
	std::vector<double> xinit_;
	std::vector<double> yinit_;
	std::vector<double> thetainit_;
	std::vector<std::vector<double>> obst_;
	double ts_ ;
	double d_;
	double safety_dist_;
    public:
	FG_eval(size_t N_,size_t m_,std::vector<double> xr_,std::vector<double> yr_,std::vector<double> thetar_,double d_,std::vector<double> xinit_,std::vector<double> yinit_,std::vector<double> thetainit_,double ts_,double safety_dist_,std::vector<std::vector<double>> obst_)
	:N_(N_),m_(m_),xr_(xr_),yr_(yr_),thetar_(thetar_),d_(d_),xinit_(xinit_),yinit_(yinit_),thetainit_(thetainit_),ts_(ts_),safety_dist_(safety_dist_),obst_(obst_){

	};
        typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;


        void operator()(ADvector& fg, const ADvector& x)
        {   assert( fg.size() == 3*N_*m_ + 3*(N_+1)*(m_+1)*(m_-2)/2.0 + 3*N_ +4 + 3*m_ + 3*(N_+1)*obst_.size()*m_ );
            assert( x.size()  == (5*(N_-1)+10)*m_ + (4*(N_-1)+8)*(m_*(m_-1)/2 + m_ * obst_.size()));

/**	m vehicles; N horizon; N+1 groups states;N+1 groups input; overall vars: (5(N-1)+10)m + (4(N-1)+8)(m(m-1)/2 + mp )
*	========= x y z linearvelocity angle
*	vehicle 1 : (x0 x1 x2 x3 x4)(x5 x6 x7 x8 x9)...(x 5(N-1)+5,x 5(N-1)+6,x 5(N-1)+7,5(N-1)+8,5(N-1)+9) start_index: 0 ;end_index:5(N-1)+9;
*	vehicle 2 : (x0 x1 x2 x3 x4)(x5 x6 x7 x8 x9)...(x 5(N-1)+5,x 5(N-1)+6,x 5(N-1)+7,5(N-1)+8,5(N-1)+9) start_index: 5(N-1)+10;end_index:5(N-1)+10+5(N-1)+9
*	...
*	vehicle m : (x0 x1 x2 x3 x4)(x5 x6 x7 x8 x9)...(x 5(N-1)+5,x 5(N-1)+6,x 5(N-1)+7,5(N-1)+8,5(N-1)+9) start_index: (5(N-1)+10)(m-1);end_index:(5(N-1)+10)(m-1)+5(N-1)+9
*	========= plane_a0 plane_a1 plane_b soft_factor (inter-vehicles seperation planes)
*	------ set var1 = (5(N-1)+10)m
*	vehicle 12 : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7 ) start_index: var1;end_index:var1+4(N-1)+7
*	vehicle 13 : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7) start_index: var1 + 4(N-1)+8;end_index:var1 + 4(N-1)+8+ 4(N-1)+7
*	...
* 	vehicle 1m : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7) start_index: var1 + (4(N-1)+8)(m-2);end_index:var1 + (4(N-1)+8)(m-2)+ 4(N-1)+7
*	------ set var2 = var1 + (4(N-1)+8)(m-1)
*	vehicle 23 : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7) start_index: var2 ;end_index:var2 + 4(N-1)+7
*	vehicle 2m : start_index: var2+ (4(N-1)+8)(m-3);end_index:var2+ (4(N-1)+8)(m-3)  + 4(N-1)+7
*	------ set var3 = var2 + (4(N-1)+8)(m-2)
*	vehicle 34 : start_index: var3;end_index:var3+4(N-1)+7
*	vehicle 3m : start_index: var3 + (4(N-1)+8)(m-4);end_index:var3+ (4(N-1)+8)(m-4) + 4(N-1)+7
*	------ set var(m-1) = var(m-2) + (4(N-1)+8)2
*	vehicle (m-1)m : start_index : var(m-1); end_index:var(m-1)+4(N-1)+7 	
*	========== vehicle-obstacles seperation planes
*	------ set start = (5(N-1)+10)m + (4(N-1)+8)m(m-1)/2
*	vehicle 11 : (a0 a1 a2 a3)...(a 4(N-1)+4,a 4(N-1)+5,a 4(N-1)+6,a 4(N-1)+7 ) start_index = start; end_index = start + 4(N-1)+7 ;
*	vehicle 12 : start_index = start + 4(N-1)+8; end_index = start+ 4(N-1)+8 + 4(N-1)+7;
*	vehicle 1p : start_index = start + (4(N-1)+8)(p-1); end_index = start+ (4(N-1)+8)(p-1)+ 4(N-1)+7;
*	------ set start1 = start+ (4(N-1)+8)p
*	vehicle 21 : start_index = start1; end_index = start1+ 4(N-1)+7;
* 	vehicle 22 : start_index = start1+ 4(N-1)+8; end_index = start1+ 4(N-1)+8 + 4(N-1)+7;
*	vehicle 2p : start_index = start1 + (4(N-1)+8)*(p-1); end_index =  start + (4(N-1)+8)*(p-1) + 4(N-1)+7;
*	-------
*	vehicle mp : end_index = (5(N-1)+10)m + (4(N-1)+8)m(m-1)/2 + (4(N-1)+8)mp - 1
**/
	    std::vector<std::vector<AD<double>>> v_states;
	    size_t step_1 = 5*(N_-1)+10;
	    for(size_t i = 0; i < m_;i++){
			std::vector<AD<double>> vv;
			for(size_t j = 0; j < N_+1;j++){
				AD<double> x_tmp1 = x[i*step_1 + j*5];
				vv.push_back(x_tmp1);
				AD<double> x_tmp2 = x[i*step_1 + j*5 + 1];
				vv.push_back(x_tmp2);
				AD<double> x_tmp3 = x[i*step_1 + j*5 + 2];
				vv.push_back(x_tmp3);
				AD<double> x_tmp4 = x[i*step_1 + j*5 + 3];
				vv.push_back(x_tmp4);
				AD<double> x_tmp5 = x[i*step_1 + j*5 + 4];
				vv.push_back(x_tmp5);
		

			}
			v_states.push_back(vv);
		};

				



	    std::vector<std::vector<std::vector<AD<double>>>> v_planes;
	    size_t var = (5*(N_-1)+10)*m_;
	    size_t start_i = var; // equal to var1 above
	    for(size_t i = 0; i < m_-1;i++){
			std::vector<std::vector<AD<double>>> first_v;
			if(i>0) start_i += (4*(N_-1)+8)*(m_-i); // equal to var2 3 4 5 ...(m-1) above
	    		for(size_t j = 0; j < m_-i-1;j++){
				std::vector<AD<double>> second_v;
				size_t start_j = start_i + j * (4*(N_-1)+8);				
				for(size_t k = 0; k < N_+1;k++){
					AD<double> x_tmp1 = x[start_j + k*4];
					second_v.push_back(x_tmp1);
					AD<double> x_tmp2 = x[start_j + k*4 + 1];
					second_v.push_back(x_tmp2);
					AD<double> x_tmp3 = x[start_j + k*4 + 2];
					second_v.push_back(x_tmp3);
					AD<double> x_tmp4= x[start_j + k*4 + 3];
					second_v.push_back(x_tmp4);


				}
				first_v.push_back(second_v);
			}
			
			v_planes.push_back(first_v);
	    }

	   
	    std::vector<std::vector<std::vector<AD<double>>>> v_obst;
	    size_t start_obst = (5*(N_-1)+10)*m_ + (4*(N_-1)+8)*m_*(m_-1)/2;
	    for(size_t i = 0; i < m_;i++){
			std::vector<std::vector<AD<double>>> ve;
			for(size_t j = 0; j < obst_.size();j++){
				std::vector<AD<double>> ob;
				for(size_t k = 0; k < N_+1;k++){
					AD<double> x_tmp1 = x[start_obst + i*obst_.size()*(N_+1)*4 + j*4*(N_+1) + k*4];
					ob.push_back(x_tmp1);
					AD<double> x_tmp2 = x[start_obst + i*obst_.size()*(N_+1)*4 + j*4*(N_+1)+ k*4 +  1];
					ob.push_back(x_tmp2);
					AD<double> x_tmp3 = x[start_obst + i*obst_.size()*(N_+1)*4 + j*4*(N_+1)+ k*4 + 2];
					ob.push_back(x_tmp3);
					AD<double> x_tmp4= x[start_obst + i*obst_.size()*(N_+1)*4 + j*4*(N_+1)+ k*4 + 3];
					ob.push_back(x_tmp4);	
					//std::cout<<"sv "<<start_obst + i*obst_.size()*(N_+1)*4 + j*4*(N_+1)+ k*4 <<" "<<start_obst + i*obst_.size()*(N_+1)*4 + j*4*(N_+1) + k*4+ 1<<\
					" "<<start_obst + i*obst_.size()*(N_+1)*4 + j*4*(N_+1)+ k*4 + 2<<" "<<start_obst + i*obst_.size()*(N_+1)*4 + j*4*(N_+1) + k*4+ 3	<<std::endl;	
				}
				ve.push_back(ob);
			}
			v_obst.push_back(ve);
		}	    
//std::cout<<v_obst.size()<<" "<<v_obst[0].size()<<" "<<v_obst[0][0].size()<<std::endl;

/**
*	
*		
*	kinematics : 
*		fg[1] = x0 + ts*x9*cos(x2) - x3;
*		fg[2] = x1 + ts*x9*sin(x2) - x4 ;
*		fg[3] = x2 + ts*tan(x10)/d - x5;
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
	    for(size_t i = 0; i < m_ ; i++){
		    for(size_t j = 0; j < N_+1;j++){
				fg[0] += (v_states[i][j*5] - xr_[i])*(v_states[i][j*5] - xr_[i])+(v_states[i][j*5+1] - yr_[i])*(v_states[i][j*5+1] - yr_[i]);
				fg[0] += v_states[i][j*5+3] *0.01*v_states[i][j*5+3] +v_states[i][j*5+4]*0.01*v_states[i][j*5+4];
				//std::cout<<"ss3 : "<<j*5<<" "<<j*5+1<<std::endl;
			}	
		}

	    // penalize soft factors of inter-vehicles
	    for(size_t i = 0; i < m_-1 ; i++){
		    for(size_t j = 0; j < m_ - i - 1;j++){
				for(size_t k = 0; k < N_+ 1;k++){
					fg[0] += (v_planes[i][j][k*4+3])*100000000*(v_planes[i][j][k*4+3]);
					//std::cout<<"ss4 : "<<k*4+3<<std::endl;
				}

			}	
		}

	    // penalize soft factors of obst-vehicles
	    for(size_t i = 0; i < m_ ; i++){
		    for(size_t j = 0; j < obst_.size() ;j++){
				for(size_t k = 0; k < N_+ 1;k++){
					fg[0] += (v_obst[i][j][k*4+3])*100000000*(v_obst[i][j][k*4+3]);
					//std::cout<<"ss4 : "<<k*4+3<<std::endl;
				}

			}	
		}

	    // penalize formation
	    for(size_t i = 0; i<N_+1 ; i++){
			fg[0] += 10.0*((v_states[0][i*5] - v_states[1][i*5])*(v_states[0][i*5] - v_states[1][i*5])+(v_states[0][i*5+1] - v_states[1][i*5+1])*(v_states[0][i*5+1] - v_states[1][i*5+1]) - 2.0)*((v_states[0][i*5] - v_states[1][i*5])*(v_states[0][i*5] - v_states[1][i*5])+(v_states[0][i*5+1] - v_states[1][i*5+1])*(v_states[0][i*5+1] - v_states[1][i*5+1]) - 2.0);

fg[0] +=10.0* ((v_states[0][i*5] - v_states[2][i*5])*(v_states[0][i*5] - v_states[2][i*5])+(v_states[0][i*5+1] - v_states[2][i*5+1])*(v_states[0][i*5+1] - v_states[2][i*5+1]) - 2.0)*((v_states[0][i*5] - v_states[2][i*5])*(v_states[0][i*5] - v_states[2][i*5])+(v_states[0][i*5+1] - v_states[2][i*5+1])*(v_states[0][i*5+1] - v_states[2][i*5+1]) - 2.0);

fg[0] += 10.0*((v_states[1][i*5] - v_states[2][i*5])*(v_states[1][i*5] - v_states[2][i*5])+(v_states[1][i*5+1] - v_states[2][i*5+1])*(v_states[1][i*5+1] - v_states[2][i*5+1]) - 4.0)*((v_states[1][i*5] - v_states[2][i*5])*(v_states[1][i*5] - v_states[2][i*5])+(v_states[1][i*5+1] - v_states[2][i*5+1])*(v_states[1][i*5+1] - v_states[2][i*5+1]) - 4.0);
		}


	    // kinematics constraints:
	    for(size_t i = 0; i < m_;i++){
		    for(size_t j = 0; j < N_;j++){
				fg[i*3*N_ + j*3 + 1] = v_states[i][j*5] + ts_ * v_states[i][j*5+3] * cos(v_states[i][j*5+2]) - v_states[i][j*5+5];
				fg[i*3*N_ + j*3 + 2] = v_states[i][j*5+1] + ts_ * v_states[i][j*5+3] * sin(v_states[i][j*5+2]) - v_states[i][j*5+6];
				fg[i*3*N_ + j*3 + 3] = v_states[i][j*5+2] + ts_ *  v_states[i][j*5+3]* tan(v_states[i][j*5+4])/d_ - v_states[i][j*5+7];
				//std::cout<<"ss5 : "<<i*3*N_ + j*3 + 1<<" "<<i*3*N_ + j*3 + 2<<" "<<i*3*N_ + j*3 + 3<<std::endl;
				//std::cout<<"ss6 : "<<j*5<<" "<<j*5+1<<" "<<j*5+2<<" "<<j*5+3<<" "<<j*5+4<<" "<<j*5+5<<" "<<j*5+6<<" "<<j*5+7<<std::endl;
			}			

		}; //fg[3Nm] is the last constraint 


					




	    // inter-vehicles avoidance constraints:

	    size_t start_avo = 3*N_*m_ + 1;
	    size_t step_avo = 0;
	    for(size_t i = 0; i < m_-1;i++){
		    if(i>0) step_avo += (N_+1) * 3 * (m_ - i); 
		    for(size_t j = 0; j < m_ - i -1;j++){
				for(size_t k = 0; k < N_+1;k++){
					fg[start_avo + step_avo +j*3*(N_+1) + k*3] = - (v_states[i][k*5] * v_planes[i][j][k*4] + v_states[i][k*5+1] * v_planes[i][j][k*4+1]) + v_planes[i][j][k*4+2] - safety_dist_ + v_planes[i][j][k*4+3];
					fg[start_avo + step_avo + j*3*(N_+1) + k*3 + 1] =  (v_states[i+j+1][k*5] * v_planes[i][j][k*4] + v_states[i+j+1][k*5+1] * v_planes[i][j][k*4+1]) - v_planes[i][j][k*4+2] - safety_dist_ + v_planes[i][j][k*4+3];
					fg[start_avo + step_avo + j*3*(N_+1) + k*3 + 2] = -(v_planes[i][j][k*4] * v_planes[i][j][k*4] + v_planes[i][j][k*4+1] * v_planes[i][j][k*4+1]) +1.0;
					//std::cout<<"ss7 : "<<start_avo + step_avo +j*3*(N_+1) + k*3<<" "<<start_avo + step_avo +j*3*(N_+1) + k*3+1<<" "<<start_avo + step_avo +j*3*(N_+1) + k*3+2<<std::endl;
					//std::cout<<"ss7: "<<k*4<<" "<<k*4+1<<" "<<k*4+2<<" "<<k*4+3<<std::endl;
					//std::cout<<"ss22: "<<k*5<<" "<<k*5+1<<std::endl;
				}
			}
	        };//fg[ 3*N_*m_ + 3*(N_+1)*(m_+1)*(m_-2)/2.0 + 3*N_ +3] is the last constraint



	   

	    size_t start_init = 0;
	    // inital constraints: 
	    if(m_ > 1){
	    		 start_init = 3*N_*m_ + 3*(N_+1)*(m_+1)*(m_-2)/2.0 + 3*N_ +4;
		}else{
			 start_init = 3*N_*m_ + 1;
		}	

	    for(size_t i = 0; i < m_;i++){
			fg[start_init + i*3] = v_states[i][0] - xinit_[i];
			fg[start_init + i*3 +1] = v_states[i][1] - yinit_[i];
			fg[start_init + i*3 +2] = v_states[i][2] - thetainit_[i];
			
		}//fg[ 3*N_*m_ + 3*(N_+1)*(m_+1)*(m_-2)/2.0 + 3*N_ +3 + 3*m_] is the last constraint




	    // obst-vehicles avoidance constraints
	    size_t start_obst_v = 3*N_*m_ + 3*(N_+1)*(m_+1)*(m_-2)/2.0 + 3*N_ +4 + 3*m_;
	//std::cout<<start_obst_v<<std::endl;
	    for(size_t i = 0; i < m_;i++){
			for(size_t j = 0; j < obst_.size();j++){
				for(size_t k = 0; k < N_+1;k++){
					fg[start_obst_v+ i*obst_.size()*(N_+1)*3 + j*(N_+1)*3 + k*3] = - (v_states[i][k*5] * v_obst[i][j][k*4] + v_states[i][k*5+1] * v_obst[i][j][k*4+1]) + v_obst[i][j][k*4+2] - safety_dist_ + v_obst[i][j][k*4+3];
					fg[start_obst_v+ i*obst_.size()*(N_+1)*3 + j*(N_+1)*3 + k*3 + 1] =  (obst_[j][0] * v_obst[i][j][k*4] + obst_[j][1] * v_obst[i][j][k*4+1]) - v_obst[i][j][k*4+2] - safety_dist_ + v_obst[i][j][k*4+3];
					fg[start_obst_v+ i*obst_.size()*(N_+1)*3 + j*(N_+1)*3 + k*3 + 2] =  -(v_obst[i][j][k*4]*v_obst[i][j][k*4] + v_obst[i][j][k*4+1]* v_obst[i][j][k*4+1]) + 1.0 ;
					//std::cout<<"ddd "<<start_obst_v+ i*obst_.size()*(N_+1)*3 + j*(N_+1)*3 + k*3<<std::endl;
					//std::cout<<"ddd "<<start_obst_v+ i*obst_.size()*(N_+1)*3 + j*(N_+1)*3 + k*3 + 1<<std::endl;
					//std::cout<<"ddd "<<start_obst_v+ i*obst_.size()*(N_+1)*3 + j*(N_+1)*3 + k*3 + 2<<std::endl;
				}

			}

		}//fg[3*N_*m_ + 3*(N_+1)*(m_+1)*(m_-2)/2.0 + 3*N_ +4 + 3*m_ + 3*(N_+1)*obst_.size()*m_ - 1] is the last constraint
	  

		

            return;
        }
    };
}

#endif 
