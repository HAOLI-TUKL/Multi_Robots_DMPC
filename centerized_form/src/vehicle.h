
#ifndef _VEHICLE_H
#define _VEHICLE_H
#include <cmath>
class Vehicle{
	private :
	double d_;
	double Ts_;
	double x_;
	double y_;
	double theta_;

	public:
	Vehicle():x_(0.0),y_(0.0),theta_(0.0),Ts_(0.1),d_(1){};
	Vehicle(double init_x,double init_y,double init_theta,double Ts,double d): x_(init_x),y_(init_y),theta_(init_theta),Ts_(Ts),d_(d){

	};
	~Vehicle(){};
	double get_x(){ return x_;}
	double get_y(){ return y_;}
	double get_theta(){ return theta_;}
	void UpdateStates(const double& v,const double& angle);
};
void Vehicle::UpdateStates(const double& v,const double& angle){
	x_ += Ts_ * v * std::cos(theta_);
	y_ += Ts_ * v * std::sin(theta_);
	theta_ += Ts_ * std::tan(angle) / d_;
	return;
};
#endif
