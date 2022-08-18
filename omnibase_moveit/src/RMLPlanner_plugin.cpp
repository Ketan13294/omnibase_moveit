#include <ros/ros.h>

#include "RMLPlanner_context.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>

#include <class_loader/class_loader.h>

#include <dynamic_reconfigure/server.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

class RMLPlanner : public planning_interface::PlannerManager{

public:

  RMLPlanner() : planning_interface::PlannerManager() {}

  virtual 
  bool 
  initialize
  ( const robot_model::RobotModelConstPtr& model, 
    const std::string& ns ){

    if (!ns.empty())
      nh = ros::NodeHandle( ns );
    context.reset( new RMLContext( model, 
				     std::string( "RML" ),
				     std::string( "omnibase_planner" ), 
				     nh ) );
    
    return true;

  }

  virtual
  bool 
  canServiceRequest
  ( const moveit_msgs::MotionPlanRequest &req ) const 
  { return true; }

  virtual std::string getDescription() const
  { return std::string( "RMLPlanner" ); }

  virtual
  void 
  getPlanningAlgorithms
  ( std::vector<std::string> &algs ) const{
    algs.resize(1);
    algs[0] = "RMLPlanner";
  }

  virtual 
  planning_interface::PlanningContextPtr 
  getPlanningContext
  ( const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes &error_code) const{
    context->setPlanningScene( planning_scene );
    context->setMotionPlanRequest( req );
    return context;
  }

  virtual 
  void 
  setPlannerConfigurations
  (const planning_interface::PlannerConfigurationMap &pconfig){}

private:
  
  ros::NodeHandle nh;
  planning_interface::PlanningContextPtr  context;

};

CLASS_LOADER_REGISTER_CLASS( RMLPlanner, planning_interface::PlannerManager );
