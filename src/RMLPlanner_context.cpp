#include "RMLPlanner_context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>


// utility function to test for a collision
bool RMLContext::is_colliding( const vertex& q ) const {
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "omnibase_planner", q );
  robotstate.update();
  
  if( getPlanningScene()->isStateColliding( robotstate, "omnibase_planner", false ) )
    {
      return true;
    }
  else
    {
      return false; }
}

// utility function to interpolate between two configurations
RMLContext::vertex RMLContext::interpolate( const RMLContext::vertex& qA,
						const RMLContext::vertex& qB,
						double t ){
  RMLContext::vertex qt( qA.size(), 0.0 );
  for( std::size_t i = 0; i < qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;
}

RMLContext::RMLContext( const robot_model::RobotModelConstPtr& robotmodel,
			    const std::string& name, 
			    const std::string& group, 
			    const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

RMLContext::~RMLContext(){}


// TODO
RMLContext::index RMLContext::select_config_from_tree( const std::vector<RMLContext::weight>& w )
{
  RMLContext::index it;
  double sum_w = 0.0;
  for(int j = 0; j<w.size(); ++j)
  {
    sum_w += w[j];
  }

  std::vector<RMLContext::weight> cummmulative_w;
  
  for(int i=0; i < w.size(); ++i)
  {
    if(cummmulative_w.empty())
    {
      cummmulative_w.push_back(w[i]/sum_w);
    }
    else
    {
      cummmulative_w.push_back((double)(w[i]/sum_w) + cummmulative_w[cummmulative_w.size()-1]);
    }
  }

  double rand_prob = (rand()%100)/100.0;
  // Use the weights to return the index of a configuration in the tree
  
  if(rand_prob <= cummmulative_w[0])
  {
    it = 0;
  }
  else
  {
    for(std::size_t i = 1 ;i < cummmulative_w.size() ;++i)
    { 
      if(cummmulative_w[i-1]<rand_prob && cummmulative_w[i]>=rand_prob)
      {it = i;}
    }
  }
  return it;
}

// TODO
RMLContext::vertex RMLContext::sample_nearby( const RMLContext::vertex& q ){
  RMLContext::vertex q_rand(q.size(),0.0);
  double q_temp = 0;
  double temp;
  // Generate a random configuration near q
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> distribution(-1,1);
  while(true)
  {
    for(int i=0; i < q.size(); ++i)
    {
      q_temp = q[i] + distribution(generator);
      
      if(i == 0 || i == 1)
      {
        if(q_temp > 8 || q_temp < -8)
        {
          q_temp = q[i] + distribution(generator);
        }
      }
      q_rand[i] = q_temp;
    }
    if(is_local_path_collision_free(q,q_rand))
    {break;}
  }
  return q_rand;
}

// TODO
bool RMLContext::is_local_path_collision_free( const RMLContext::vertex& q,
						 const RMLContext::vertex& q_rand ){
    // return true/false if the local path between q and q_rand is collision free
    for(double t=0.01; t<=1; t+=0.01)
    {
      RMLContext::vertex q_int = interpolate(q,q_rand,t);
      if(is_colliding(q_int))
      {
        return false;
        break;
      }
    }
  return true;
}

bool RMLContext::test_connection(const RMLContext::vertex& q,
                                      const RMLContext::vertex& q_goal,
                                      RMLContext::index& connection)
{
  bool connected = 0;
  if(is_local_path_collision_free(q,q_goal))
  {
    connection = 1;
    return true;
  }
  return false;
}

void RMLContext::add_new_branch_to_tree(const int& parent,
                                          RMLContext::vertex& weights,
                                          RMLContext::sym_vertex& V)
{
  V.push_back(parent);
  if(parent == -1)
  {weights.push_back(1.0);}
  else
  {weights.push_back(0.5);}

  if(parent >= 0)
  {
    double wt = weights[parent];
    weights[parent] = wt/(wt+1);
  }
}


RMLContext::sym_vertex RMLContext::search_path( const RMLContext::sym_vertex& V,
 					      const index& idx_init,
					      const index& idx_goal)
                {
  RMLContext::sym_vertex P;
  
  // find and return a path (ordered sequence of configurations) between the indices
  if(idx_init == idx_goal)
  {
    P.push_back(idx_init);
  }
  else
  {
    P.push_back(idx_goal);

    RMLContext::index curr_node = idx_goal;
   
    while(V[curr_node] != -1)
    {
      P.push_back(V[curr_node]);
      curr_node = V[curr_node];
    }
  }
  return P;
}


RMLContext::path RMLContext::est( const RMLContext::vertex& q_init,
 				      const RMLContext::vertex& q_goal ){
  
  state_init.clear();
  weight_init.clear();
  tree_init.clear();

  RMLContext::path P;
  bool connected = 0;
  goal_connection = -1;
  
  vertex q_init_temp = q_init;
  state_init.push_back(q_init_temp);
  add_new_branch_to_tree(-1,weight_init,tree_init);
  
  if(test_connection(state_init[state_init.size()-1],q_goal,goal_connection))
  {
    goal_connection = state_init.size()-1;
    connected = 1;
  }

  // TODO implement EST algorithm and return the path (an ordered sequence of configurations).
  while(!connected)
  {
    int i1 = select_config_from_tree(weight_init);
    q_init_temp = sample_nearby(state_init[i1]);
    state_init.push_back(q_init_temp);
    add_new_branch_to_tree(i1,weight_init,tree_init);
    
    if(test_connection(state_init[state_init.size()-1],q_goal,goal_connection))
    {
      goal_connection = state_init.size()-1;
      connected = 1;
      break;
    }
  }
ROS_INFO("Found a path");

RMLContext::sym_vertex f1 = search_path(tree_init,0,goal_connection);
for(int i = f1.size()-1; i >= 0; --i)
  {
    P.push_back(state_init[f1[i]]);
  }
  P.push_back(q_goal);
return P;
}

// This is the method that is called each time a plan is requested
// You should not need to modify code below this (but it's ok of you absolutely need to).
bool RMLContext::solve( planning_interface::MotionPlanResponse &res ){
  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, getGroupName()));
  res.trajectory_->clear();
  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  vertex q_init, q_goal;
  for( size_t i=0; i<3; i++ )
  {
    q_goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    q_init.push_back(request_.start_state.joint_state.position[i]);
  }
  // start the timer
  ros::Time begin = ros::Time::now();

  path P = est( q_init, q_goal );
  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "omnibase_planner", q_init );
  
  for( std::size_t i=1; i<P.size(); i++ ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i-1], P[i], t );
      robotstate.setJointGroupPositions( "omnibase_planner", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
    }
  }  
  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

bool RMLContext::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void RMLContext::clear(){}

bool RMLContext::terminate(){return true;}
