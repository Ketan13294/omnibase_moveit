#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <random>


MOVEIT_CLASS_FORWARD( RMLContext );

class RMLContext : public planning_interface::PlanningContext {

public:

  typedef double weight;
  typedef std::size_t index;
  typedef std::vector<double> vertex;
  typedef std::vector<int> sym_vertex;  // vetor for indices
  typedef std::vector<vertex> path;
  typedef std::vector<sym_vertex> sym_path; // vector of vector of indices

  RMLContext( const robot_model::RobotModelConstPtr& model,
		const std::string &name,
		const std::string& group,
		const ros::NodeHandle &nh = ros::NodeHandle("~") );
  virtual ~RMLContext();

  virtual bool solve( planning_interface::MotionPlanResponse &res );
  virtual bool solve( planning_interface::MotionPlanDetailedResponse &res );

  virtual void clear();
  virtual bool terminate();

  /**
     Utility method
     Determine if a configuration collides with obstacles.

     \input q Robot configuration
     \return True if the robot collides with an obstacle. False otherwise.
  */
  bool is_colliding( const vertex& q ) const;

  /**
     Utility method
     Interpolate between two configurations.

     \input qA The first configuration
     \input qB The second configuration
     \input t The interpolation parameter t=[0,1]
     \return The interpolated configuration
  */
  vertex interpolate( const vertex& qA, const vertex& qB, double t );

  /**
     Sample a configuration according to a probability density function provided by the 
     configuration weights.

     \input weights A vector of weights. w[i] is the weight of ith configuration
     \return The index of the randomly chosen configuration
  */
  index select_config_from_tree( const std::vector<weight>& w );
  
  /**
     Create a random sample. The returned vertex represents a collision-free
     configuration (i.e. 6 joints) of the robot.
     \return  A collision-free configuration
  */
  vertex sample_nearby( const vertex& q );

  /**
     Determine if a straight line path is collision-free between two configurations.
     \input q_near The first configuration
     \input q_rand The second configuration
     \return True if the path is collision-free. False otherwise.
  */
  bool is_local_path_collision_free( const vertex& q, const vertex& q_rand );
  
   /**
   * @brief
   * Determine if there is connection possible between the new node and the tree provided
   * \input q The new vertex generated
   * \input V_op The list of states of the other side
   * \input connection If connected the index of the contact node on other tree
   * \return True if trees are connected else false and
   *  also udpates the connection variable with the point of connection on the tree. 
   */
  bool test_connection(const RMLContext::vertex& q,
                                       const RMLContext::vertex& q_goal,
                                       RMLContext::index& connection);

   /**
   * @brief 
   * The tree adds the generated node to the tree
   * \input q THe generated input
   * \input parent The index of the parent of the generated node
   * \input treeA The tree structure
   * \result treeA The structure is updated in the function
   */
  void add_new_branch_to_tree(const int& parent,
                              RMLContext::vertex& weights,
                              RMLContext::sym_vertex& V);

  /**
   * @brief 
   * 
   * Once the goal configuration has been added to the tree. Search the tree to find and return the 
   * path between the root (q_init) and the goal (q_goal).
   * \input q_init The root configuration
   * \input q_goal The goal configuration
   * \return The path (a sequence of configurations) between q_init and q_goal.
  */
  sym_vertex search_path( const sym_vertex& V,
		   const index& idx_init,
		   const index& idx_goal );

  /**
     TODO
     
     This is the main RRT algorithm that adds vertices/edges to a tree and searches for a path.
     \input q_init The initial configuration
     \input q_goal The goal configuration
     \return The path between q_init and q_goal
  */
  path est( const vertex& q_init, const vertex& q_goal );
  
 protected:

  robot_model::RobotModelConstPtr robotmodel;
  // the connection point on both trees
  RMLContext::index init_connection;
  RMLContext::index goal_connection;

  // vector of actual states
  RMLContext::path state_init;
  RMLContext::path state_goal;

  // Weight vectors
  RMLContext::vertex weight_init;
  RMLContext::vertex weight_goal;

  // Connected to node vector of vector of index
  RMLContext::sym_vertex tree_init;
  RMLContext::sym_vertex tree_goal;

  std::vector<double> workspace_limits;
};

