#include "frontier_viewpoint_optimizer/frontier_viewpoint_optimizer.hpp"
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <set>
#include <ros2top/ros2top.hpp>
#include <nlohmann/json.hpp>

namespace frontier_viewpoint_optimizer {
namespace {
constexpr double PI=3.14159265358979323846;
double rad(double x) {
	return x*PI/180.0;
}
double wrap(double x) {
	while (x>PI) {
		x-=2*PI;
	}
	while (x<-PI) {
		x+=2*PI;
	}
	return x;
}
Eigen::Vector3d ep(const geometry_msgs::msg::Point &p) {
	return {p.x,p.y,p.z};
}
geometry_msgs::msg::Point gp(const Eigen::Vector3d&p) {
	geometry_msgs::msg::Point q;
	q.x=p.x();
	q.y=p.y();
	q.z=p.z();
	return q;
}
std::string joinTopics(const std::initializer_list<std::string> &topics) {
	std::string joined;
	for (const auto &topic:topics) {
		if (!joined.empty()) {
			joined += ", ";
		}
		joined += topic;
	}
	return joined;
}
}
FrontierViewpointOptimizer::FrontierViewpointOptimizer() : Node("frontier_viewpoint_optimizer_node"){
	frontier_topic_=param<std::string>("frontier_topic","/frontier_segments");
	map_topic_=param<std::string>("octomap_topic","/octomap_binary");
	odom_topic_=param<std::string>("odometry_topic","/Odometry");
	result_topic_=param<std::string>("optimized_topic","/frontier_viewpoints");
	marker_topic_=param<std::string>("fov_marker_topic","/frontier_viewpoints/fov_markers");
	frame_override_=param<std::string>("frame_id_override","");
	hfov_=rad(param<double>("horizontal_fov_deg",360.0));
	vmin_=rad(param<double>("vertical_fov_min_deg",-88.5));
	vmax_=rad(param<double>("vertical_fov_max_deg",46.5));
	rmin_=param<double>("min_range",0.2); rmax_=param<double>("max_range",40.0);
	sensor_offset_={param<double>("sensor_offset_x",0.0),param<double>("sensor_offset_y",0.0),param<double>("sensor_offset_z",0.0)};
	radial_step_=param<double>("candidate_radial_step",2.0);
	radial_layers_=param<int>("candidate_radial_layers",3);
	azimuth_samples_=param<int>("candidate_azimuth_samples",16); altitude_offsets_=param<std::vector<double> >("candidate_altitude_offsets",{-1,0,1,2});
	min_offset_=param<double>("minimum_frontier_offset",1.0);
	max_offset_=param<double>("maximum_frontier_offset",8.0);
	collision_radius_=param<double>("uav_collision_radius",0.55);
	collision_step_=param<double>("collision_check_step",0.2);
	reach_step_=param<double>("reachability_check_step",0.25);
	require_map_=param<bool>("require_octomap_for_collision_check",true);
	max_candidates_=param<int>("max_candidate_count",160);
	yaw_samples_=param<int>("yaw_samples",12);
	area_weight_=param<double>("frontier_area_weight",3.0);
	volume_weight_=param<double>("new_volume_weight",0.7);
	altitude_weight_=param<double>("altitude_bias_weight",0.25);
	altitude_ref_=param<double>("altitude_reference_z",1.5);
	altitude_excess_=param<double>("altitude_excess_scale",2.0);
	excess_weight_=param<double>("excessive_altitude_penalty_weight",1.2);
	ray_azimuth_samples_=param<int>("volume_azimuth_samples",48);
	ray_elevation_samples_=param<int>("volume_elevation_samples",13);
	ray_step_=param<double>("volume_ray_step",0.4);
	unknown_cap_=param<double>("volume_unknown_depth_cap",8.0);
	enable_refine_=param<bool>("enable_candidate_refining",true);
	refine_iterations_=param<int>("refinement_iterations",2);
	refine_xy_=param<double>("refinement_xy_step",0.5);
	refine_z_=param<double>("refinement_z_step",0.35);
	refine_yaw_=rad(param<double>("refinement_yaw_step_deg",10.0));
	marker_segments_=param<int>("fov_azimuth_segments",48);
	marker_alpha_=param<double>("fov_marker_alpha",0.16);
	enable_diagnostics_=param<bool>("enable_diagnostics",false);
	visualize_markers_as_arrows_=param<bool>("visualize_markers_as_arrows",false);

	frontier_sub_=create_subscription<FrontierArray>(frontier_topic_,rclcpp::QoS(1).reliable().transient_local(), std::bind(&FrontierViewpointOptimizer::frontierCb,this,std::placeholders::_1));

	map_sub_=create_subscription<octomap_msgs::msg::Octomap>(map_topic_,rclcpp::QoS(1).reliable().transient_local(), std::bind(&FrontierViewpointOptimizer::mapCb,this,std::placeholders::_1));

	odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom_topic_,10, std::bind(&FrontierViewpointOptimizer::odomCb,this,std::placeholders::_1));

	result_pub_=create_publisher<frontier_viewpoint_optimizer::msg::OptimizedFrontierArray>(result_topic_,rclcpp::QoS(1).reliable().transient_local());

	marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_,rclcpp::QoS(1).reliable().transient_local());

	publish_timer_=create_wall_timer(std::chrono::milliseconds(1000),std::bind(&FrontierViewpointOptimizer::publishCb,this));

	heartbeat_timer_=create_wall_timer(std::chrono::seconds(3),std::bind(&FrontierViewpointOptimizer::heartbeatCb,this));

	registerTop();
	RCLCPP_INFO(get_logger(),"Incremental frontier viewpoint optimizer started.");
}

FrontierViewpointOptimizer::~FrontierViewpointOptimizer(){
	unregisterTop();
}
void FrontierViewpointOptimizer::registerTop(){
	nlohmann::json j={{"description","Incremental 3D frontier viewpoint optimizer"},
		{"version","0.1.0"},
		{"node_type","exploration_viewpoint_optimizer"},
		{"topics_subscribed",joinTopics({frontier_topic_,map_topic_,odom_topic_})},
		{"topics_published",joinTopics({result_topic_,marker_topic_})}};
	ros2top_registered_=ros2top::register_node("/"+std::string(get_name()),j);
	if (!ros2top_registered_) {
		RCLCPP_WARN(get_logger(),"ros2top registration failed.");
	}
}
void FrontierViewpointOptimizer::unregisterTop(){
	if (ros2top_registered_) {
		ros2top::unregister_node("/"+std::string(get_name()));
	}
}
void FrontierViewpointOptimizer::heartbeatCb(){
	if (ros2top_registered_) {
		ros2top::heartbeat("/"+std::string(get_name()));
	}
}

void FrontierViewpointOptimizer::mapCb(const octomap_msgs::msg::Octomap::SharedPtr m){
	auto *a=octomap_msgs::msgToMap(*m);
  	if (!a) {
		RCLCPP_WARN(get_logger(),"OctoMap conversion failed"); 
		return;
	}
	auto *o=dynamic_cast<octomap::OcTree*>(a); 
  	if (!o) {
		delete a; 
		RCLCPP_WARN(get_logger(),"Expected OcTree"); 
		return;
	}
	tree_.reset(o); 
  	if (frame_override_.empty()) {
		frame_=m->header.frame_id;
	}
	processPending();
}
void FrontierViewpointOptimizer::odomCb(const nav_msgs::msg::Odometry::SharedPtr m){
	robot_={m->pose.pose.position.x,m->pose.pose.position.y,m->pose.pose.position.z}; have_odom_=true;
	if (frame_override_.empty()) {
		frame_=m->header.frame_id;
	}
	processPending();
}
void FrontierViewpointOptimizer::frontierCb(const FrontierArray::SharedPtr m){
	have_frontiers_=true; 
  	if (frame_override_.empty()) {
		frame_=m->header.frame_id;
	}
	else { 
		frame_=frame_override_;
	}
	std::set<uint32_t> live;
	for (const auto&f:m->frontiers) {
		live.insert(f.id); 
		frontiers_[f.id]=f;
	}
	for (auto it=frontiers_.begin(); it!=frontiers_.end();) {
		if (!live.count(it->first)) {
			results_.erase(it->first); 
			it=frontiers_.erase(it);
		}
		else { 
			++it;
		}
	}
	active_ids_.assign(live.begin(),live.end());
	processPending();
}
void FrontierViewpointOptimizer::processPending(){
	if (!tree_||!have_odom_) {
		return;
	}
	for (auto &entry:frontiers_) {
		auto &f=entry.second;
		if (results_.find(f.id)==results_.end()||f.recent) {
			Candidate c=optimize(f);
			if (c.score>-1e20) {
				results_[f.id]=c;
			}
			else { 
				results_.erase(f.id);
			}
			f.recent=false;
		}
	}
}
std::vector<Eigen::Vector3d> FrontierViewpointOptimizer::samplePositions(const frontier_segmentation::msg::Frontier&f)const {
	std::vector<Eigen::Vector3d> anchors{ep(f.centroid)};
	const size_t n=f.voxels.size(), take=std::min<size_t>(32,n);
	for (size_t i=0; i<take; i++) anchors.push_back(ep(f.voxels[i*n/std::max<size_t>(1,take)]));
	std::vector<Eigen::Vector3d> out;
	for (const auto&a:anchors) for (int l=0; l<radial_layers_; l++) {
			double r=std::clamp(min_offset_+l*radial_step_,min_offset_,std::min(max_offset_,std::max(min_offset_,rmax_-collision_radius_)));
			for (int k=0; k<std::max(4,azimuth_samples_); k++) for (double dz:altitude_offsets_) {
					double th=2*PI*k/std::max(4,azimuth_samples_);
					out.emplace_back(a.x()+r*cos(th),a.y()+r*sin(th),f.centroid.z+dz);
					if ((int)out.size()>=max_candidates_) {
						return out;
					}
				}
		}
	return out;
}
bool FrontierViewpointOptimizer::freeSphere(const Eigen::Vector3d&p)const {
	if (!tree_) {
		return !require_map_;
	}
	double step=std::max(tree_->getResolution(),collision_step_);
	int n=std::max(1,(int)ceil(collision_radius_/step));
	for (int x=-n; x<=n; x++) for (int y=-n; y<=n; y++) for (int z=-n; z<=n; z++) {
				Eigen::Vector3d q=p+Eigen::Vector3d(x*step,y*step,z*step); if ((q-p).norm()>collision_radius_) {
					continue;
				}
				auto*node=tree_->search(q.x(),q.y(),q.z()); if (!node||tree_->isNodeOccupied(node)) {
					return false;
				}
			}
	return true;
}
bool FrontierViewpointOptimizer::reachable(const Eigen::Vector3d&a,const Eigen::Vector3d&b)const {
	double d=(b-a).norm(); int n=std::max(1,(int)ceil(d/std::max(0.05,reach_step_)));
	for (int i=0; i<=n; i++) if (!freeSphere(a+(b-a)*(double(i)/n))) {
			return false;
		}
	return true;
}
bool FrontierViewpointOptimizer::inFov(const Eigen::Vector3d&p,const Eigen::Vector3d&o,double yaw)const {
	Eigen::Vector3d d=p-o; double r=d.norm(); if (r<rmin_||r>rmax_) {
		return false;
	}
	double az=wrap(atan2(d.y(),d.x())-yaw),el=atan2(d.z(),hypot(d.x(),d.y()));
	return std::abs(az)<=hfov_*0.5+1e-9&&el>=vmin_&&el<=vmax_;
}
double FrontierViewpointOptimizer::frontierArea(const frontier_segmentation::msg::Frontier&f,const Eigen::Vector3d&o,double yaw)const {
	if (f.voxels.empty()) {
		return 0;
	}
	double a=0,unit=std::max(0.0,(double)f.surface_area_proxy)/f.voxels.size();
	for (const auto&v:f.voxels) {Eigen::Vector3d p=ep(v); if (!inFov(p,o,yaw)) {
									 continue;
								 }
								 if (tree_) {
									 octomap::point3d origin(o.x(),o.y(),o.z()),end(p.x(),p.y(),p.z()),hit;
									 octomap::OcTreeKey origin_key,end_key;
									 if (!tree_->coordToKeyChecked(origin,origin_key)
										 || !tree_->coordToKeyChecked(end,end_key)) {
										 continue;
									 }
									 octomap::point3d dir=end-origin; dir.normalize();
									 if (tree_->castRay(origin,dir,hit,true,std::max(0.0,(p-o).norm()-tree_->getResolution()))) {
										 continue;
									 }
								 }
								 a+=unit;} return a;
}
double FrontierViewpointOptimizer::unknownVolume(const Eigen::Vector3d&o,double yaw)const {
	if (!tree_) {
		return 0;
	}
	int na=std::max(4,ray_azimuth_samples_),ne=std::max(2,ray_elevation_samples_);
	double hf=std::min(hfov_,2*PI),da=hf/na,de=(vmax_-vmin_)/(ne-1),sum=0;
	for (int ia=0; ia<na; ia++) for (int ie=0; ie<ne; ie++) {
			double az=yaw-hf/2+(ia+0.5)*da,el=vmin_+ie*de;
			Eigen::Vector3d dir(cos(el)*cos(az),cos(el)*sin(az),sin(el));
			double unknown=0,known=0; bool entered=false;
			for (double r=rmin_; r<=rmax_; r+=ray_step_) {
				Eigen::Vector3d p=o+dir*r; auto*n=tree_->search(p.x(),p.y(),p.z());
				if (n) {
					if (tree_->isNodeOccupied(n)) {
						break;
					}
					if (entered) {
						break;
					}
					known+=ray_step_;
				}
				else {entered=true; unknown+=ray_step_; if (unknown>=unknown_cap_) {
						  break;
					  }}
			}
			// Differential solid-angle volume, only for the first unknown interval.
			double r0=rmin_+known,r1=r0+unknown;
			sum+=(da*de/3.0)*std::max(0.0,r1*r1*r1-r0*r0*r0);
		}return sum;
}
Candidate FrontierViewpointOptimizer::evaluate(const frontier_segmentation::msg::Frontier&f,const Eigen::Vector3d&p,double yaw)const {
	Candidate c; c.p=p; c.yaw=yaw; Eigen::Vector3d o=p+sensor_offset_;
	c.area=frontierArea(f,o,yaw); c.volume=unknownVolume(o,yaw);
	double dz=p.z()-altitude_ref_;
	double altitude_penalty=altitude_weight_*std::max(0.0,dz);
	double excess_penalty=excess_weight_*pow(std::max(0.0,dz-altitude_excess_),2);
	c.score=area_weight_*c.area+volume_weight_*c.volume-altitude_penalty-excess_penalty; return c;
}
Candidate FrontierViewpointOptimizer::optimize(const frontier_segmentation::msg::Frontier&f){
	Candidate best; double seed=atan2(f.centroid.y-robot_.y(),f.centroid.x-robot_.x());
	auto ps=samplePositions(f);
	// An ideal 360-degree scanner is yaw invariant: evaluate one representative yaw.
	std::vector<double> ys; 
  if (hfov_>=2*PI-1e-6) {
		ys={seed};
	}
	else 
  { 
    for (int i=0; i<std::max(1,yaw_samples_); i++) {
      ys.push_back(-PI+2*PI*i/std::max(1,yaw_samples_)); 
    }
  }
	for (const auto&p:ps) {if (!freeSphere(p)||!reachable(robot_,p)) {
							   continue;
						   }
						   for (double y:ys) {auto c=evaluate(f,p,y); if (c.score>best.score) {
												  best=c;
											  }}}
	if (enable_refine_&&best.score>-1e20) {
		refine(f,best);
	}
	return best;
}
void FrontierViewpointOptimizer::refine(const frontier_segmentation::msg::Frontier&f,Candidate&best)const {
	double xy=refine_xy_,z=refine_z_,ya=refine_yaw_;
	for (int it=0; it<refine_iterations_; it++) {
		Candidate b=best; std::vector<Eigen::Vector3d>d={{xy,0,0},{-xy,0,0},{0,xy,0},{0,-xy,0},{xy,xy,0},{xy,-xy,0},{-xy,xy,0},{-xy,-xy,0},{0,0,z},{0,0,-z}};
		for (auto v:d) {auto p=best.p+v; if (!freeSphere(p)||!reachable(robot_,p)) {
							continue;
						}
						for (double dy:{0.0,ya,-ya}) {auto c=evaluate(f,p,wrap(best.yaw+dy)); if (c.score>b.score) {
														  b=c;
													  }}}
		best=b; xy*=0.5; z*=0.5; ya*=0.5;
	}
}
visualization_msgs::msg::MarkerArray FrontierViewpointOptimizer::makeMarkers()const {
	visualization_msgs::msg::MarkerArray out;
	visualization_msgs::msg::Marker clear; 
	clear.header.frame_id=frame_; 
	clear.header.stamp=now();
	clear.action=visualization_msgs::msg::Marker::DELETEALL; out.markers.push_back(clear);
	int id=0; 
	int n=std::max(8,marker_segments_); 
	double hf=std::min(hfov_,2*PI);
	for (auto fid:active_ids_) {
		auto it=results_.find(fid); 
		if (it==results_.end()) {
			continue;
		}
		const auto&c=it->second;
		Eigen::Vector3d o=c.p+sensor_offset_; visualization_msgs::msg::Marker m;
		m.header.frame_id=frame_; m.header.stamp=now(); m.ns="frontier_lidar_fov"; m.id=id++;
		m.action=visualization_msgs::msg::Marker::ADD;
		const double color_position=static_cast<double>(fid%10)/9.0;
		m.color.r=1.0;
		m.color.g=1.0-color_position;
		m.color.b=0.0;
		m.color.a=visualize_markers_as_arrows_ ? 1.0 : marker_alpha_;
		if (visualize_markers_as_arrows_) {
			m.type=visualization_msgs::msg::Marker::ARROW;
			m.pose.position=gp(c.p);
			m.pose.orientation.z=sin(c.yaw/2);
			m.pose.orientation.w=cos(c.yaw/2);
			m.scale.x=std::min(3.0,std::max(0.5,rmax_*0.1));
			m.scale.y=0.18;
			m.scale.z=0.18;
			out.markers.push_back(m);
			continue;
		}
		m.type=visualization_msgs::msg::Marker::TRIANGLE_LIST;
		m.pose.orientation.w=1; m.scale.x=m.scale.y=m.scale.z=1;
		auto tri=[&](Eigen::Vector3d a,Eigen::Vector3d b,Eigen::Vector3d d){
			m.points.push_back(gp(a)); 
			m.points.push_back(gp(b)); 
			m.points.push_back(gp(d));
		};
		for (int i=0; i<n; i++) {
			double a0=c.yaw-hf/2+hf*i/n,a1=c.yaw-hf/2+hf*(i+1)/n;
			auto pt=[&](double a,double e){
				return o+rmax_*Eigen::Vector3d(cos(e)*cos(a),cos(e)*sin(a),sin(e));
			};
			auto p00=pt(a0,vmin_),p10=pt(a1,vmin_),p01=pt(a0,vmax_),p11=pt(a1,vmax_);
			tri(o,p00,p10); tri(p00,p01,p11); tri(p00,p11,p10); tri(o,p10,p01);} out.markers.push_back(m);
	} 
	return out;
}
void FrontierViewpointOptimizer::publishCb(){
	if (!have_frontiers_) {
		return;
	}
	frontier_viewpoint_optimizer::msg::OptimizedFrontierArray out;
	out.header.stamp=now(); out.header.frame_id=frame_;
	for (auto id:active_ids_) {
		auto it=results_.find(id);
		if (it==results_.end()) {
			continue;
		}
		frontier_viewpoint_optimizer::msg::OptimizedFrontier v;
		v.id=id;
		v.viewpoint.header=out.header;
		v.viewpoint.pose.position=gp(it->second.p);
		v.viewpoint.pose.orientation.z=sin(it->second.yaw/2);
		v.viewpoint.pose.orientation.w=cos(it->second.yaw/2);
		v.observable_frontier_area=it->second.area;
		v.expected_new_volume=it->second.volume;
		v.information_gain=it->second.area+it->second.volume;
		v.objective_score=it->second.score;
		out.viewpoints.push_back(v);
		if (enable_diagnostics_) {
			RCLCPP_INFO(get_logger(),
				"Frontier %u chosen pose: x=%.3f y=%.3f z=%.3f yaw=%.3f score=%.3f",
				id,it->second.p.x(),it->second.p.y(),it->second.p.z(),
				it->second.yaw,it->second.score);
		}
	}
	result_pub_->publish(out);
	marker_pub_->publish(makeMarkers());
}
}
int main(int argc,char**argv){
	rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<frontier_viewpoint_optimizer::FrontierViewpointOptimizer>()); rclcpp::shutdown(); return 0;
}
