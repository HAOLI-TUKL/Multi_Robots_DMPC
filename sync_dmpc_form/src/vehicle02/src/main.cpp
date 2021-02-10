/** 
* This code doesnt consider the time delay caused by the communication and the solving process;
* The failure to solve is considered but it is assumed the time it costs is ignorable even if it fails to solve;
**/



#include "batch_solver.h"
#include <iostream>
#include "ros/ros.h"
#include "vehicle.h"
#include "utility.h"
#include <visualization_msgs/Marker.h>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include "mymsg/neighborpos.h"
#include "std_msgs/String.h"


size_t N = 15;
size_t m = 2;// represent the num of neighbors for this vehicle
double Hz = 5.0;
size_t update_shift = 0;

double xr;
double yr;
double thetar ;


std::mutex neig_mtx;


double xinit ;
double yinit ;
double thetainit ;

// ******protected by neig_mtx
std::vector<std::vector<std::vector<double>>> neig;
 

std::vector<std::vector<double>> pre_states(N+1,std::vector<double>(3.0,0.0));
std::vector<std::vector<double>> pre_inputs(N+1,std::vector<double>(2.0,0.0));


bool solve_success = false;

bool first_solution_v1 = false;
bool first_solution_v2 = false;


double d = 1.0;
double ts = 1.0 / Hz;
double safety_dist = 0.5;


std::shared_ptr<Vehicle> vehicle; 
std::vector<std::vector<double>> obst;

std::shared_ptr<BatchSolver> bs(new BatchSolver(N,xr, yr, thetar, d, xinit,yinit, thetainit, ts, safety_dist,obst,neig));

ros::Publisher vehicle_pub;// for visulization
ros::Publisher markerArray;
ros::Publisher neig_pub;// tells other robots my pos
ros::Subscriber sub1;
ros::Subscriber sub2;
//ros::Subscriber testsub;

void UpdateVisualize();
//void RunSolver();
void UpdateNeighborsPos();
void Initialize(ros::NodeHandle& n);
void NeighborCallback1(const mymsg::neighborpos& msg);
void NeighborCallback2(const mymsg::neighborpos& msg);


int main(int argc,char* argv[]){
	ros::init(argc,argv,"vehicle02");
	ros::NodeHandle n;
   	ros::Rate loop_rate(50);
	Initialize(n);



	std::thread sim_thread(&UpdateVisualize);
	sim_thread.detach();



	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
};

void Initialize(ros::NodeHandle& n){
	
	#if 0
	// test setting 1
	xinit=5.0;yinit=0.0;thetainit=3.14;
	xr=-5.0;yr=0.0;thetar=0.0;
	std::vector<double> obst1 = {0.0,0.0};
	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{-5.0,0.0});
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{0.0,5.0});
	#endif

	#if 0
	// test setting 2
	xinit=-5.0;yinit=-5.0;thetainit=3.14/4.0;
	xr=5.0;yr=5.0;thetar=0.0;
	std::vector<double> obst1 = {0.0,0.0};
	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{-5.0,5.0});
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{5.0,-5.0});
	#endif

	#if 0
	// test setting 3
	xinit=-5.0;yinit=-1.0;thetainit=0.00001;
	xr=5.0;yr=-1.0;thetar=0.0;
	std::vector<double> obst1 = {0.0,0.0};
	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{-5.0,1.0});
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{5.0,0.0});
	#endif

	#if 0
	// test setting for formation 1
	xinit=-2.0;yinit=-6.0;thetainit=0.00001;
	xr=-1.0;yr=4.0;thetar=0.0;
	std::vector<double> obst1 = {0.0,0.0};
	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{0.0,-3.0});
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{2.0,6.0});
	#endif 

	#if 1
	// test setting for formation 2
	xinit=-4.0;yinit=-8.0;thetainit=3.14;
	xr=-1.0;yr=4.0;thetar=0.0;
	std::vector<double> obst1 = {0.0,0.0};
	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{0.0,-3.0});
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{4.0,8.0});
	#endif 

	#if 0
	// test setting for formation 3
	xinit=-0.0;yinit=-7.0;thetainit=0.1;
	xr=-1.0;yr=4.0;thetar=0.0;
	std::vector<double> obst1 = {0.0,0.0};
	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{-3.0,-7.0});
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{3.0,-7.0});
	#endif 


	obst.push_back(obst1);
	neig.push_back(neig1);	
	neig.push_back(neig2);	
	bs->set_neighbors(neig,neig_mtx);
	bs->set_obst_(obst);
	bs->set_initial_states(xinit,yinit,thetainit);
	bs->set_ref_states(xr,yr,thetar);



	std::shared_ptr<Vehicle> vehicle_tmp(new Vehicle(xinit, yinit, thetainit, ts, d));
	vehicle = vehicle_tmp;

	vehicle_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        markerArray = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
	neig_pub= n.advertise<mymsg::neighborpos>("neig2pos", 10);
        sub1 = n.subscribe("neig1pos", 1000, NeighborCallback1);
        sub2 = n.subscribe("neig3pos", 1000, NeighborCallback2);
        // testsub = n.subscribe("chatter", 1000, chatterCallback);
	
};
void UpdateVisualize(){

	ros::Rate loop_rate_sim(Hz);

	ros::spinOnce();
	loop_rate_sim.sleep();

	while(ros::ok()){

/** 
* in the instant i ,this vehicle solves the optimization problem using assumed trajectory obtained in 
* the last iteration. Then it sends the solution immediately. Then it sleeps for a while.
* The states is updated in the instant i+1;
**/
		ObstRviz(obst,safety_dist,markerArray);

		bs->set_initial_states(xinit,yinit,thetainit);
			
		bs->set_neighbors(neig,neig_mtx);
		bs->Solve(pre_states,pre_inputs,solve_success);

		if(solve_success){
			update_shift = 0;

			std::vector<double> pre_x;
			std::vector<double> pre_y;
			for(int i = 0 ; i < pre_states.size(); i++){
				pre_x.push_back(pre_states[i][0]);
				pre_y.push_back(pre_states[i][1]);
			}
   			mymsg::neighborpos msg;
   			msg.xpos = pre_x;
   			msg.ypos = pre_y;
    			msg.time_stamp = ros::Time::now().toSec();
			neig_pub.publish(msg);
		}else{// if fail to solve, publish the shifted pre_states
			update_shift++;	
			std::cout<<" ***publish previous states : "<< update_shift <<" ***"<<std::endl;		
			if(update_shift<N){
				std::vector<double> pre_x;
				std::vector<double> pre_y;
				for(int i = update_shift ; i < pre_states.size(); i++){
					pre_x.push_back(pre_states[i][0]);
					pre_y.push_back(pre_states[i][1]);
				}
				
 				for(int i = pre_states.size() ; i < pre_states.size()+update_shift; i++){
					pre_x.push_back(0.0);
					pre_y.push_back(0.0);
				}
				mymsg::neighborpos msg;
   				msg.xpos = pre_x;
   				msg.ypos = pre_y;
    				msg.time_stamp = ros::Time::now().toSec();
				neig_pub.publish(msg);
	

			}else{
				std::cout<<" *** !!! no more previous states !!! *** "<<std::endl;
				return;
			}

		}

		ros::spinOnce();// send the solution ASAP after the solving
		loop_rate_sim.sleep();

		while(first_solution_v1 == false || first_solution_v2 == false){};

		if(solve_success){
			solve_success = false;
			vehicle->UpdateStates(pre_inputs[0][0],pre_inputs[0][1]);

		}else{// if fail to solve, use shifted input in the last iteration

			if(update_shift<N){

				vehicle->UpdateStates(pre_inputs[update_shift][0],pre_inputs[update_shift][1]);
			}else{

				return;
			}
		}

		xinit = vehicle->get_x();
		yinit = vehicle->get_y();
		thetainit = vehicle->get_theta();
		VehicleRviz(xinit,yinit,thetainit,safety_dist,vehicle_pub);
		HeadingRviz(xinit,yinit,thetainit,safety_dist,vehicle_pub);
		TrajRviz(pre_states, safety_dist,markerArray);
		

	}


};

void NeighborCallback1(const mymsg::neighborpos& msg)
{

	std::vector<double> x = msg.xpos;
	std::vector<double> y = msg.ypos;
	std::lock_guard<std::mutex> lk(neig_mtx);
	for(int i = 1; i< neig[0].size();i++){
		neig[0][i-1][0] = x[i];
		neig[0][i-1][1] = y[i];
		

	}
	neig[0][neig[0].size()-1][0] = neig[0][neig[0].size()-2][0];
	neig[0][neig[0].size()-1][1] = neig[0][neig[0].size()-2][1];	
	first_solution_v1 = true;
};

void NeighborCallback2(const mymsg::neighborpos& msg)
{

	std::vector<double> x = msg.xpos;
	std::vector<double> y = msg.ypos;
	std::lock_guard<std::mutex> lk(neig_mtx);
	for(int i = 1; i< neig[0].size();i++){
		neig[1][i-1][0] = x[i];
		neig[1][i-1][1] = y[i];
		

	}
	neig[1][neig[1].size()-1][0] = neig[1][neig[1].size()-2][0];
	neig[1][neig[1].size()-1][1] = neig[1][neig[1].size()-2][1];	
	first_solution_v2 = true;
};


