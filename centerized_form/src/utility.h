#ifndef _UTILITY_H
#define _UTILITY_H
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
void VehicleRviz(const double& x, const double& y, const double& theta,const ros::Publisher& vehicle_pub){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = sin(theta / 2.0) * 0.0;
    marker.pose.orientation.y = sin(theta / 2.0) * 0.0;
    marker.pose.orientation.z = sin(theta / 2.0) * 1.0;
    marker.pose.orientation.w = cos(theta / 2.0);
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    vehicle_pub.publish(marker);
};
void ObstRviz(const std::vector<std::vector<double>>& obst,const double& safety_dist,const ros::Publisher& obst_pub){
        visualization_msgs::MarkerArray ma;
        for (int i = 0; i < obst.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = obst[i][0];
            marker.pose.position.y = obst[i][1];
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = safety_dist*2.0;
            marker.scale.y = safety_dist*2.0;
            marker.scale.z = 0.1;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            ma.markers.push_back(marker);
        }
        obst_pub.publish(ma);
        ma.markers.clear();	
};
void VehicleRviz(const std::vector<double> x,const std::vector<double> y,const std::vector<double> theta,const double safety_dist,const ros::Publisher& pub ){

        visualization_msgs::MarkerArray ma;
        for (int i = 0; i < x.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = i+100;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x[i];
            marker.pose.position.y = y[i];
            marker.pose.position.z = 0;
            marker.pose.orientation.x = sin(theta[i] / 2.0) * 0.0;
            marker.pose.orientation.y = sin(theta[i] / 2.0) * 0.0;
            marker.pose.orientation.z = sin(theta[i] / 2.0) * 1.0;
            marker.pose.orientation.w = cos(theta[i] / 2.0);
            marker.scale.x = 1.0;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 0.7f*(1.0 - i/x.size());
            marker.color.g = 0.3f;
            marker.color.b = 1.0f* i/x.size();
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            ma.markers.push_back(marker);
        }
        pub.publish(ma);
        ma.markers.clear();

   	for (int i = 0; i < x.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = i+1000;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x[i];
            marker.pose.position.y = y[i];
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = safety_dist * 2.0;
            marker.scale.y = safety_dist * 2.0;
            marker.scale.z = 0.1;
            marker.color.r = 0.7f*(1.0 - i/x.size());
            marker.color.g = 0.3f;
            marker.color.b = 1.0f* i/x.size();
            marker.color.a = 0.4;
            marker.lifetime = ros::Duration();
            ma.markers.push_back(marker);
        }
        pub.publish(ma);
        ma.markers.clear();
   
}

#endif 
