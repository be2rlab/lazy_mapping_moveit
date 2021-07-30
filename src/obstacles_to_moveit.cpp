#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stdio.h>
#include <geometry_msgs/PointStamped.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "std_msgs/Float32MultiArray.h"

#include <lazy_mapping_moveit/Marker.h>
#include <lazy_mapping_moveit/Markers.h>

#include <XmlRpcValue.h>


static const std::string PLANNING_GROUP = "manipulator";

class MoveitObstacles {
  private:
    ros::NodeHandle nh;
    // ros::ServiceServer obstacle_adder;
    ros::Subscriber markers_sub;

    std::vector<moveit_msgs::CollisionObject> obstacles;
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_ptr;
    const moveit::core::JointModelGroup* joint_model_group;

    // tf2_ros::Buffer buffer;
    // tf2_ros::TransformListener listener;

    bool is_recived;
    
    std::string name_obj;
    double *a;  // [x_dim, y_dim, z_dim,  x, y, z,   i, j, k, w]
    std::vector< std::vector<double> > obstacles_params; // [ [id, x_dim, y_dim, z_dim], ... ]

  public:
    MoveitObstacles() : nh("~") {
      std::cout << 3 << std::endl;

      // read obstacles parameters `[id, 3d_dimentions]`
      XmlRpc::XmlRpcValue obstacles_params_list;
      ros::param::get("/lazy_mapping_moveit/obstacles", obstacles_params_list);

      // parse config.yaml with obstacles
      if(obstacles_params_list.getType() == XmlRpc::XmlRpcValue::Type::TypeArray && obstacles_params_list.size() > 0){
        for (int i = 0; i < obstacles_params_list.size(); ++i) {
          XmlRpc::XmlRpcValue sublist = obstacles_params_list[i];
          if(sublist.getType() == XmlRpc::XmlRpcValue::Type::TypeArray && sublist.size() > 0){
            std::vector<double> obst_i;
            for (int j = 0; j < sublist.size(); ++j) {
              XmlRpc::XmlRpcValue element = sublist[j];
              if (element.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                double x = static_cast<double>(element);
                obst_i.push_back(x);
                // ROS_INFO("%f", x);
              }
            }
            obstacles_params.push_back(obst_i);
          }
        }
      }

      // test parsing config.yaml
      // for (int i = 0; i < obstacles_params.size(); ++i) {
      //   for (int j = 0; j < obstacles_params[0].size(); ++j) {
      //     ROS_INFO("%f", obstacles_params[i][j]);
      //   }
      // }

      std::cout << 4 << std::endl;
      
      move_group_interface_ptr = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
      
      std::cout << 5 << std::endl;

      joint_model_group = move_group_interface_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      std::cout << 6 << std::endl;

      // obstacle_adder = n.advertiseService("/add_obstacle", &MoveitObstacles::obstacle_adder_handler, this);
      markers_sub = nh.subscribe("/markers", 1, &MoveitObstacles::markers_callback, this);  // subscribe to the arcode_detector_node
      std::cout << 7 << std::endl;


      is_recived = false;   
      a = new double[10];
      ROS_INFO("Obstacles2MoveIt run!");

      // add_floor();
      add_static_obs();

    }

    ~MoveitObstacles() {
      delete move_group_interface_ptr;
      delete a;
    }


    void add_static_obs() {

      // FLOOR 1
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.5;
      primitive.dimensions[primitive.BOX_Y] = 1;
      primitive.dimensions[primitive.BOX_Z] = 0.05;

      // Define a pose for the box (specified relative to frame_id)
      geometry_msgs::Pose box_pose;
      box_pose.orientation.x = 0;
      box_pose.orientation.y = 0;
      box_pose.orientation.z = 0;
      box_pose.orientation.w = 0;
      box_pose.position.x = 0.45;
      box_pose.position.y = 0.0;
      box_pose.position.z = 0.07;  

      moveit_msgs::CollisionObject collision_object;
      collision_object.id = "floor1";
      collision_object.header.frame_id = move_group_interface_ptr->getPlanningFrame();

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      obstacles.push_back(collision_object);
      ROS_INFO_NAMED("MoveitObstacles", "Add an object into the world");

      planning_scene_interface.addCollisionObjects(obstacles);

      // FLOOR 2
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.5;
      primitive.dimensions[primitive.BOX_Y] = 1;
      primitive.dimensions[primitive.BOX_Z] = 0.05;

      // Define a pose for the box (specified relative to frame_id)
      box_pose.orientation.x = 0;
      box_pose.orientation.y = 0;
      box_pose.orientation.z = 0;
      box_pose.orientation.w = 0;
      box_pose.position.x = -0.42;
      box_pose.position.y = 0.0;
      box_pose.position.z = 0.07;  

      collision_object.id = "floor2";
      collision_object.header.frame_id = move_group_interface_ptr->getPlanningFrame();

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      obstacles.push_back(collision_object);
      ROS_INFO_NAMED("MoveitObstacles", "Add an object into the world");

      planning_scene_interface.addCollisionObjects(obstacles);

      // FLOOR 3
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 1.0;
      primitive.dimensions[primitive.BOX_Y] = 0.5;
      primitive.dimensions[primitive.BOX_Z] = 0.05;

      // Define a pose for the box (specified relative to frame_id)
      box_pose.orientation.x = 0;
      box_pose.orientation.y = 0;
      box_pose.orientation.z = 0;
      box_pose.orientation.w = 0;
      box_pose.position.x = 0.0;
      box_pose.position.y = 0.44;
      box_pose.position.z = 0.07;  

      collision_object.id = "floor3";
      collision_object.header.frame_id = move_group_interface_ptr->getPlanningFrame();

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      obstacles.push_back(collision_object);
      ROS_INFO_NAMED("MoveitObstacles", "Add an object into the world");

      planning_scene_interface.addCollisionObjects(obstacles);

      // FLOOR 4
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 1.0;
      primitive.dimensions[primitive.BOX_Y] = 0.5;
      primitive.dimensions[primitive.BOX_Z] = 0.05;

      // Define a pose for the box (specified relative to frame_id)
      box_pose.orientation.x = 0;
      box_pose.orientation.y = 0;
      box_pose.orientation.z = 0;
      box_pose.orientation.w = 0;
      box_pose.position.x = 0.0;
      box_pose.position.y = -0.46;
      box_pose.position.z = 0.07;  

      collision_object.id = "floor4";
      collision_object.header.frame_id = move_group_interface_ptr->getPlanningFrame();

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      obstacles.push_back(collision_object);
      ROS_INFO_NAMED("MoveitObstacles", "Add an object into the world");

      planning_scene_interface.addCollisionObjects(obstacles);


    }


		void markers_callback(const lazy_mapping_moveit::Markers::ConstPtr &msg) {
      
      for (int i = 0; i < msg->markers.size(); ++i) {

        bool is_id_exist = false;
        for (int j = 0; j < obstacles_params.size(); ++j) {
          if ( ((int) msg->markers[i].id) == obstacles_params[j][0]) { // check if the `id` is exist in config
            a[0] = obstacles_params[j][1];
            a[1] = obstacles_params[j][2];
            a[2] = obstacles_params[j][3];
            is_id_exist = true;
            break;
          }
        }

        if (!is_id_exist) {
          ROS_WARN("Excuse me, but I can't find the id: %i in the config.yaml. Using the hardcoded obstacle dims", (int) msg->markers[i].id);
          a[0] = 0.1;
          a[1] = 0.1;
          a[2] = 0.1;
        }

        a[3] =  msg->markers[i].pose.position.x;
        a[4] =  msg->markers[i].pose.position.y;
        a[5] =  msg->markers[i].pose.position.z;
        
        a[6] = msg->markers[i].pose.orientation.x;
        a[7] = msg->markers[i].pose.orientation.y;
        a[8] = msg->markers[i].pose.orientation.z;
        a[9] = msg->markers[i].pose.orientation.w;

        // test if subsribeing is ok
        // ROS_INFO("### %d", msg->markers.size());
        // for (int i = 0; i < 10; i++) {
        //   ROS_INFO("%f ", a[i]);
        // }
        // ROS_INFO("\n");

        add_obstacle(std::to_string(msg->markers[i].id));
      }

		}

    // bool obstacle_adder_handler(std_msgs::Float32MultiArray::Request  &req, beginner_tutorials::AddTwoInts::Response &res) {
    //   return true;
    // }
    void add_floor() {
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 2;
      primitive.dimensions[primitive.BOX_Y] = 2;
      primitive.dimensions[primitive.BOX_Z] = 0.1;

      // Define a pose for the box (specified relative to frame_id)
      geometry_msgs::Pose box_pose;
      box_pose.orientation.x = 0;
      box_pose.orientation.y = 0;
      box_pose.orientation.z = 1;
      box_pose.orientation.w = 0;
      box_pose.position.x = 0;
      box_pose.position.y = 0;
      box_pose.position.z = -0.05;  

      moveit_msgs::CollisionObject collision_object;
      collision_object.id = "floor";
      collision_object.header.frame_id = move_group_interface_ptr->getPlanningFrame();

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      obstacles.push_back(collision_object);
      ROS_INFO_NAMED("MoveitObstacles", "Add the floor");

      planning_scene_interface.addCollisionObjects(obstacles);
    }

    void add_obstacle(std::string obstacle_name) {
        //Препятствие
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = a[0];
        primitive.dimensions[primitive.BOX_Y] = a[1];
        primitive.dimensions[primitive.BOX_Z] = a[2];

        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.x = a[6];
        box_pose.orientation.y = a[7];
        box_pose.orientation.z = a[8];
        box_pose.orientation.w = a[9];
        box_pose.position.x = a[3];
        box_pose.position.y = a[4];
        box_pose.position.z = a[5];  

      // search if the obstacle exists in `obstacles` list
      bool is_obstacle_exist = false;
      for (int i = 0; i < obstacles.size(); ++i) {
        if (obstacles[i].id == obstacle_name) { 

          // update pose
          obstacles[i].header.frame_id = move_group_interface_ptr->getPlanningFrame();
          obstacles[i].primitives[0] = primitive;
          obstacles[i].primitive_poses[0] = box_pose;

          // ROS_INFO_NAMED("MoveitObstacles", "Update the object pose into the world");

          is_obstacle_exist = true;
          break;
        }
      }

      if (!is_obstacle_exist) {
        moveit_msgs::CollisionObject collision_object;
        collision_object.id = obstacle_name;
        collision_object.header.frame_id = move_group_interface_ptr->getPlanningFrame();

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        obstacles.push_back(collision_object);
        ROS_INFO_NAMED("MoveitObstacles", "Add an object into the world");
      }

      planning_scene_interface.addCollisionObjects(obstacles);
    }

    void spin() {
      ros::Rate R(1);
			while (nh.ok()) {
        ROS_INFO("Count of obstacles: %i", (int) obstacles.size());
        R.sleep();
      }
    }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_obstacles_node");
  std::cout << 1 << std::endl;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::cout << 2 << std::endl;

  MoveitObstacles moveit_obstacles;
  // moveit_obstacles.spin();
  ros::waitForShutdown();

  ros::shutdown();
  
  return 0;
}