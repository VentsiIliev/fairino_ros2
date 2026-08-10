#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "erob_moveit_runtime/srv/compute_linked_lin.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>

using ComputeLinkedLin = erob_moveit_runtime::srv::ComputeLinkedLin;

namespace {
constexpr int32_t OK=1, BAD=-1, IK_FAIL=-2, FK_FAIL=-3, STEP_FAIL=-4, SPAN_FAIL=-5, COLLISION=-10;
double nowS(){using C=std::chrono::steady_clock;return std::chrono::duration<double>(C::now().time_since_epoch()).count();}
Eigen::Isometry3d tf(const geometry_msgs::msg::Transform& m){Eigen::Quaterniond q(m.rotation.w,m.rotation.x,m.rotation.y,m.rotation.z);if(q.norm()<1e-12)q=Eigen::Quaterniond::Identity();q.normalize();Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.linear()=q.toRotationMatrix();t.translation()=Eigen::Vector3d(m.translation.x,m.translation.y,m.translation.z);return t;}
Eigen::Isometry3d tf(const geometry_msgs::msg::Pose& m){Eigen::Quaterniond q(m.orientation.w,m.orientation.x,m.orientation.y,m.orientation.z);if(q.norm()<1e-12)q=Eigen::Quaterniond::Identity();q.normalize();Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.linear()=q.toRotationMatrix();t.translation()=Eigen::Vector3d(m.position.x,m.position.y,m.position.z);return t;}
double maxDelta(const std::vector<double>& a,const std::vector<double>& b){double d=0;for(size_t i=0;i<std::min(a.size(),b.size());++i)d=std::max(d,std::abs(a[i]-b[i]));return d;}
double oriDeg(const Eigen::Isometry3d& a,const Eigen::Isometry3d& b){Eigen::Quaterniond qa(a.rotation()),qb(b.rotation());qa.normalize();qb.normalize();return 2.0*std::acos(std::clamp(std::abs(qa.dot(qb)),0.0,1.0))*180.0/M_PI;}
double stepFor(const Eigen::Isometry3d& a,const Eigen::Isometry3d& b,double cap){const double d=(b.translation()-a.translation()).norm(),o=oriDeg(a,b);double s=0.010;if(!(d<0.005&&o>0.5)){int n=std::max(2,std::min(6,(int)std::ceil(d/0.01)));s=std::clamp(d/(double)n,0.0005,0.025);}if(cap>0)s=std::min(s,cap);return std::max(0.0005,s);}
bool same(const moveit::core::RobotState& a,const moveit::core::RobotState& b,const moveit::core::JointModelGroup* g){std::vector<double>x,y;a.copyJointGroupPositions(g,x);b.copyJointGroupPositions(g,y);return maxDelta(x,y)<=1e-12;}
}

class LinkedLinHelperNode:public rclcpp::Node{
public:
 LinkedLinHelperNode():Node("linked_lin_helper"){RCLCPP_INFO(get_logger(),"Linked LIN helper starting...");}
 void initialize(){auto n=shared_from_this();loader_=std::make_shared<robot_model_loader::RobotModelLoader>(n);model_=loader_->getModel();if(!model_)throw std::runtime_error("Linked LIN helper failed to load robot model");psm_=std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(n,loader_,"linked_lin_planning_scene_monitor");if(!psm_->getPlanningScene())throw std::runtime_error("Linked LIN helper failed to create PlanningSceneMonitor");psm_->startSceneMonitor();psm_->startWorldGeometryMonitor();if(!psm_->requestPlanningSceneState("/get_planning_scene"))RCLCPP_WARN(get_logger(),"Could not request initial planning scene; continuing with monitored updates");srv_=create_service<ComputeLinkedLin>("/compute_linked_lin",[this](const std::shared_ptr<ComputeLinkedLin::Request> q,std::shared_ptr<ComputeLinkedLin::Response> r){handle(q,r);});RCLCPP_INFO(get_logger(),"Service '/compute_linked_lin' created");RCLCPP_INFO(get_logger(),"Linked LIN helper ready");}
private:
 rclcpp::Service<ComputeLinkedLin>::SharedPtr srv_;robot_model_loader::RobotModelLoaderPtr loader_;moveit::core::RobotModelPtr model_;planning_scene_monitor::PlanningSceneMonitorPtr psm_;
 void fail(const std::shared_ptr<ComputeLinkedLin::Response>& r,int32_t code,const std::string& msg,uint32_t seg=std::numeric_limits<uint32_t>::max()){r->success=false;r->error_code=code;r->message=msg;r->failed_segment_index=seg;r->failed_index=seg;RCLCPP_WARN(get_logger(),"Linked LIN rejected: %s",msg.c_str());}
 std::vector<double> seed(const moveit::core::JointModelGroup* g,const sensor_msgs::msg::JointState& js,const std::shared_ptr<ComputeLinkedLin::Response>& r){std::vector<double> q;for(const auto& n:g->getVariableNames()){auto it=std::find(js.name.begin(),js.name.end(),n);if(it==js.name.end()){fail(r,BAD,"seed_state missing joint "+n);return{};}size_t i=std::distance(js.name.begin(),it);if(i>=js.position.size()){fail(r,BAD,"seed_state position missing for joint "+n);return{};}q.push_back(js.position[i]);}return q;}
 bool metadata(const ComputeLinkedLin::Request& q,const std::shared_ptr<ComputeLinkedLin::Response>& r){size_t n=q.target_poses.size();if(!n){fail(r,BAD,"linked LIN request has no target poses");return false;}if((!q.labels.empty()&&q.labels.size()!=n)||q.velocities.size()!=n||q.accelerations.size()!=n||q.blend_radii.size()!=n){fail(r,BAD,"linked LIN per-segment metadata size mismatch");return false;}return true;}
 bool validateQ(const ComputeLinkedLin::Request& q,const moveit::core::JointModelGroup* g,const std::vector<double>& start,const std::vector<double>& prev,const std::vector<double>& cur,size_t seg,size_t point,const std::shared_ptr<ComputeLinkedLin::Response>& r){
  if(cur.size()!=prev.size()||cur.size()!=start.size()){fail(r,BAD,"linked LIN joint-size mismatch",seg);return false;}
  double ds=maxDelta(prev,cur);r->max_joint_step_rad=std::max(r->max_joint_step_rad,ds);if(q.max_joint_step_rad>0&&ds>q.max_joint_step_rad){std::ostringstream s;s<<"joint step "<<ds<<" exceeds "<<q.max_joint_step_rad<<" at segment "<<seg<<" point "<<point;fail(r,STEP_FAIL,s.str(),seg);return false;}
  std::unordered_set<std::string> full(q.full_turn_joint_names.begin(),q.full_turn_joint_names.end());const auto& names=g->getVariableNames();double ms=0,me=0;for(size_t j=0;j<cur.size();++j){double d=std::abs(cur[j]-start[j]);ms=std::max(ms,d);me=std::max(me,d);bool ft=j<names.size()&&full.count(names[j]);double sl=ft?q.full_turn_max_joint_span_rad:q.max_joint_span_rad,el=ft?q.full_turn_max_endpoint_delta_rad:q.max_endpoint_delta_rad;if(sl>0&&d>sl){fail(r,SPAN_FAIL,"linked LIN joint span limit exceeded",seg);return false;}if(el>0&&d>el){fail(r,SPAN_FAIL,"linked LIN endpoint delta limit exceeded",seg);return false;}}r->max_joint_span_rad=std::max(r->max_joint_span_rad,ms);r->max_endpoint_delta_rad=std::max(r->max_endpoint_delta_rad,me);return true;
 }
 bool validateFk(const ComputeLinkedLin::Request& q,const Eigen::Isometry3d& expected,const moveit::core::RobotState& s,size_t seg,const std::shared_ptr<ComputeLinkedLin::Response>& r){Eigen::Isometry3d actual=s.getGlobalLinkTransform(q.link_name)*tf(q.tool_transform);double pe=(expected.translation()-actual.translation()).norm()*1000.0,oe=oriDeg(expected,actual);r->max_fk_position_error_mm=std::max(r->max_fk_position_error_mm,pe);r->max_fk_orientation_error_deg=std::max(r->max_fk_orientation_error_deg,oe);if(q.fk_position_tolerance_mm>0&&pe>q.fk_position_tolerance_mm){fail(r,FK_FAIL,"linked LIN FK position tolerance exceeded",seg);return false;}if(q.fk_orientation_tolerance_deg>0&&oe>q.fk_orientation_tolerance_deg){fail(r,FK_FAIL,"linked LIN FK orientation tolerance exceeded",seg);return false;}return true;}
 void handle(const std::shared_ptr<ComputeLinkedLin::Request> q,std::shared_ptr<ComputeLinkedLin::Response> r){
  double t0=nowS();r->success=false;r->error_code=BAD;r->requested_segment_count=q->target_poses.size();r->requested_pose_count=r->requested_segment_count;r->solved_segment_count=r->solved_pose_count=0;r->failed_segment_index=r->failed_index=std::numeric_limits<uint32_t>::max();r->max_fk_position_error_mm=r->max_fk_orientation_error_deg=r->max_joint_step_rad=r->max_joint_span_rad=r->max_endpoint_delta_rad=r->planning_time_s=r->validation_time_s=r->total_time_s=0;
  if(!metadata(*q,r)){r->total_time_s=nowS()-t0;return;}if(q->group_name.empty()||q->link_name.empty()){fail(r,BAD,"linked LIN request missing group/link name");r->total_time_s=nowS()-t0;return;}auto* g=model_->getJointModelGroup(q->group_name);if(!g||!model_->hasLinkModel(q->link_name)){fail(r,BAD,"unknown MoveIt group/link");r->total_time_s=nowS()-t0;return;}auto* link=model_->getLinkModel(q->link_name);
  moveit::core::RobotState state(model_);state.setToDefaultValues();auto global_start=seed(g,q->seed_state,r);if(global_start.empty()){r->total_time_s=nowS()-t0;return;}state.setJointGroupPositions(g,global_start);state.update();if(!state.satisfiesBounds(g)){fail(r,BAD,"seed_state violates joint bounds");r->total_time_s=nowS()-t0;return;}r->trajectory.joint_trajectory.joint_names=g->getVariableNames();
  std::unique_ptr<planning_scene_monitor::LockedPlanningSceneRO> scene;if(q->avoid_collisions){scene=std::make_unique<planning_scene_monitor::LockedPlanningSceneRO>(psm_);if(!(*scene)){fail(r,BAD,"PlanningScene is unavailable");r->total_time_s=nowS()-t0;return;}}
  moveit::core::GroupStateValidityCallbackFn valid=[&](moveit::core::RobotState* s,const moveit::core::JointModelGroup*,const double*){return !q->avoid_collisions||!(*scene)->isStateColliding(*s,q->group_name,false);};
  size_t total_points=0;Eigen::Isometry3d work=tf(q->workobject_transform),tool=tf(q->tool_transform);
  for(size_t seg=0;seg<q->target_poses.size();++seg){
   Eigen::Isometry3d expected=tf(q->target_poses[seg]);if(q->use_workobject_transform)expected=work*expected;Eigen::Isometry3d target=expected*tool.inverse();Eigen::Isometry3d start_link=state.getGlobalLinkTransform(q->link_name);double eef_step=stepFor(start_link,target,q->cartesian_step_m);EigenSTL::vector_Isometry3d w;w.push_back(target);moveit::core::RobotState segment_start(state);std::vector<std::shared_ptr<moveit::core::RobotState>> path;double ps=nowS();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
   double fraction=moveit::core::CartesianInterpolator::computeCartesianPath(&state,g,path,link,w,true,moveit::core::MaxEEFStep(eef_step),moveit::core::JumpThreshold::disabled(),valid);
#pragma GCC diagnostic pop
   double plan_s=nowS()-ps;r->planning_time_s+=plan_s;r->segment_planning_time_s.push_back(plan_s);RCLCPP_INFO(get_logger(),"Linked LIN segment %zu/%zu: label='%s' step=%.6fm points=%zu fraction=%.6f vel=%.3f acc=%.3f blendR=%.3fmm time=%.3fs",seg+1,q->target_poses.size(),q->labels.empty()?"":q->labels[seg].c_str(),eef_step,path.size(),fraction,q->velocities[seg],q->accelerations[seg],q->blend_radii[seg],plan_s);
   if(fraction<0.999||path.empty()){fail(r,IK_FAIL,"linked LIN Cartesian segment incomplete",seg);r->total_time_s=nowS()-t0;return;}std::vector<std::shared_ptr<moveit::core::RobotState>> segment;if(!same(segment_start,*path.front(),g))segment.push_back(std::make_shared<moveit::core::RobotState>(segment_start));segment.insert(segment.end(),path.begin(),path.end());if(segment.size()<2){fail(r,IK_FAIL,"linked LIN segment produced fewer than two states",seg);r->total_time_s=nowS()-t0;return;}
   double vs=nowS();std::vector<double> first,last;segment.front()->copyJointGroupPositions(g,first);segment.back()->copyJointGroupPositions(g,last);if(maxDelta(first,last)<=1e-10){fail(r,IK_FAIL,"linked LIN segment is a no-op",seg);r->total_time_s=nowS()-t0;return;}std::vector<double> prev=first;for(size_t i=0;i<segment.size();++i){auto& s=*segment[i];s.update();if(!s.satisfiesBounds(g)){fail(r,BAD,"linked LIN state violates joint bounds",seg);r->total_time_s=nowS()-t0;return;}std::vector<double> cur;s.copyJointGroupPositions(g,cur);if(i>0&&!validateQ(*q,g,global_start,prev,cur,seg,i,r)){r->total_time_s=nowS()-t0;return;}prev=cur;}if(!validateFk(*q,expected,*segment.back(),seg,r)){r->total_time_s=nowS()-t0;return;}r->validation_time_s+=nowS()-vs;
   for(const auto& sp:segment){std::vector<double> pos;sp->copyJointGroupPositions(g,pos);trajectory_msgs::msg::JointTrajectoryPoint p;p.positions=std::move(pos);r->trajectory.joint_trajectory.points.push_back(std::move(p));++total_points;}r->segment_point_counts.push_back(segment.size());r->segment_boundary_indices.push_back(total_points-1);r->solved_segment_count=seg+1;r->solved_pose_count=r->solved_segment_count;state=*segment.back();state.update();
  }
  r->success=true;r->error_code=OK;r->message="ok";r->total_time_s=nowS()-t0;RCLCPP_INFO(get_logger(),"Linked LIN success: segments=%u points=%zu boundaries=%zu total=%.3fs planning=%.3fs validation=%.3fs",r->requested_segment_count,r->trajectory.joint_trajectory.points.size(),r->segment_boundary_indices.size(),r->total_time_s,r->planning_time_s,r->validation_time_s);
 }
};

int main(int argc,char** argv){rclcpp::init(argc,argv);auto n=std::make_shared<LinkedLinHelperNode>();try{n->initialize();rclcpp::spin(n);}catch(const std::exception& e){RCLCPP_FATAL(n->get_logger(),"Linked LIN helper initialization failed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
