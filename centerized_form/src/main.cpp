
#include "batch_solver.h"
#include <iostream>
#include "ros/ros.h"
#include "vehicle.h"
#include "utility.h"
#include <visualization_msgs/Marker.h>
#include <memory>
size_t N = 10;
size_t m = 3;
size_t Hz = 10;
size_t shift = 0;
std::vector<double> xr;
std::vector<double> yr;
std::vector<double> thetar ;
double d = 1.0;
std::vector<double> xinit ;
std::vector<double> yinit ;
std::vector<double> thetainit ;
double ts = 0.1;
double safety_dist = 0.5;
bool solve_success = false;
std::vector<std::shared_ptr<Vehicle>> vehicles; 
std::vector<std::vector<double>> obst;
std::shared_ptr<BatchSolver> bs(new BatchSolver(N, m,xr, yr, thetar, d, xinit,yinit, thetainit, ts, safety_dist,obst));
std::vector<std::vector<std::vector<double>>> pre_states(m,std::vector<std::vector<double>>(N+1,std::vector<double>(3,0.0)));
std::vector<std::vector<std::vector<double>>> pre_inputs(m,std::vector<std::vector<double>>(N+1,std::vector<double>(2,0.0)));
//ros::Publisher vehicle_pub;
ros::Publisher markerArray;

void RunMPC();
void UpdateNeighborsPos();
void Initialize(ros::NodeHandle& n);


int main(int argc,char* argv[]){
	ros::init(argc,argv,"centerilized");
	ros::NodeHandle n;
   	ros::Rate loop_rate(Hz);
	Initialize(n);
	//std::cout<<"test"<<std::endl;
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
		RunMPC();
		//exit(0);
	}
	return 0;
};

void Initialize(ros::NodeHandle& n){
	
/**	double step_angle = 3.14*2/m;
	double radius = 3.0;
	for(size_t i = 0; i <m;i++){
		xinit.push_back(radius*cos(step_angle*i));
		yinit.push_back(radius*sin(step_angle*i));
		thetainit.push_back(step_angle*i - 3.14);
		xr.push_back(radius*cos(step_angle*i + 3.14));
		yr.push_back(radius*sin(step_angle*i + 3.14));
		thetar.push_back(step_angle*i + 3.14);	

	}**/
	xinit.push_back(0.0);yinit.push_back(1.0);thetainit.push_back(3.14/2.0);
	xinit.push_back(-3.0);yinit.push_back(-3.0);thetainit.push_back(3.14/2.0);
	xinit.push_back(3.0);yinit.push_back(-3.0);thetainit.push_back(3.14/4.0);
	//xinit.push_back(3.0);yinit.push_back(0.0);thetainit.push_back(-3.14);
	//xinit.push_back(0.0);yinit.push_back(-3.0);thetainit.push_back(3.14/2.0);
	bs->set_initial_states(xinit,yinit,thetainit);



	xr.push_back(0.0);yr.push_back(8.0);thetar.push_back(0.0);
	xr.push_back(-1.0);yr.push_back(7.0);thetar.push_back(0.0);
	xr.push_back(1.0);yr.push_back(7.0);thetar.push_back(0.0);
	//xr.push_back(-3.0);yr.push_back(0.0);thetar.push_back(0.0);
	//xr.push_back(0.0);yr.push_back(2.0);thetar.push_back(0.0);
	bs->set_ref_states(xr,yr,thetar);

	std::vector<double> obst1 = {0.0,4.0};
	//obst.push_back(obst1);
	//obst1 = {0.0,1.0};
	//obst.push_back(obst1);
	//obst1 = {0.0,-1.0};
	obst.push_back(obst1);
	bs->set_obst_(obst);


	for(size_t i = 0; i < m ; i++){
		std::shared_ptr<Vehicle> v(new Vehicle(xinit[i], yinit[i], thetainit[i], ts, d));
		vehicles.push_back(v);
	}


	//vehicle_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        markerArray = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

	
};


void RunMPC(){
	ObstRviz(obst,safety_dist,markerArray);

	if(solve_success){
		for(int i = 0; i<m ; i++){
			vehicles[i]->UpdateStates(pre_inputs[i][0][0],pre_inputs[i][0][1]);
		} 
		solve_success = false;
		shift = 0;
	}else{
		shift++;
		std::cout<<" ***use old input : "<< shift <<" ***"<<std::endl;
		if(shift<N){
		
			for(int i = 0; i<m ; i++){
				vehicles[i]->UpdateStates(pre_inputs[i][0][0],pre_inputs[i][0][1]);
			}
		}else{
			std::cout<<" *** !!! no more old input !!! *** "<<std::endl;
			return;
		}
	}


	for(int i = 0; i<m ; i++){
		xinit[i] = vehicles[i]->get_x();
		yinit[i] = vehicles[i]->get_y();
		thetainit[i] = vehicles[i]->get_theta();
		}
	bs->set_initial_states(xinit,yinit,thetainit);
	VehicleRviz(xinit,yinit,thetainit,safety_dist,markerArray);
	solve_success = bs->Solve(pre_states,pre_inputs);
	
	for(int i = 0 ; i<m; i++) {
		for(int j = 0; j < N+1;j++){
			//std::cout<<"car "<<i<<" step "<<j<<" : "<< pre_inputs[i][j][0]<<" "<< pre_inputs[i][j][1]<<std::endl;

		}

	};
	//exit(0);
	return ;

};

