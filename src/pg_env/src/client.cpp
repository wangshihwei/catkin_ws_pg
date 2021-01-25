#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/ModelStates.h>
#include <cstdlib>
#include <vector>
#include <algorithm>
//#include <typeinfo>
//#include <unistd.h>

class ENV {
    private:
    int counter;
    ros::ServiceClient client;
    ros::Subscriber sub_scan;
    ros::Subscriber model_states;
    ros::Publisher pub_cmd_vel;
    geometry_msgs::Twist actionVector;
    gazebo_msgs::ModelStates modelStatus;
    std::vector<float> lidar_range = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    std::vector<float> scan_range = {0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1};

    float robot_x = -0.8 ; 
    float robot_y = 0.8 ;
    float prev_robot_x = -0.8 ;
    float prev_robot_y = 0.8 ;
    float goal_x = 0.75;
    float goal_y = 0.85;

    public:
    ENV(ros::NodeHandle *nh) {
        client = nh->serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
	pub_cmd_vel = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        sub_scan = nh->subscribe("/scan", 1000, &ENV::getLidarCallback, this);
        model_states = nh->subscribe("gazebo/model_states", 1000, &ENV::getModelStatesCallback, this);
    }
    void getLidarCallback(const sensor_msgs::LaserScan& scan_msg){
        std::cout<<"lidarCB"<<std::endl;
        for(int v=0;v<scan_msg.ranges.size();v++)
	{
	    lidar_range[v] = scan_msg.ranges[v];
            std::cout<<v<<":"<<lidar_range[v]<<std::endl;
        }
    }
    void getModelStatesCallback(const gazebo_msgs::ModelStates& modelStatusMsg){
        //std::cout<<"gmsCB"<<std::endl;
        //std::cout<<"x:"<<modelStatusMsg.pose[2].position.x<<"y:"<<modelStatusMsg.pose[2].position.y<<std::endl;
        modelStatus = modelStatusMsg ;
    }
    auto getLaserScan(){
        for(int k=0;k<lidar_range.size();k++)
	{
            //need to think about 'inf' 
            scan_range[k] = lidar_range[k];
            //std::cout<<k<<":"<<scan_range[k]<<std::endl;
	}
        return scan_range;
    }
    void getRobotStatus(float &x,float &y){
        x = modelStatus.pose[2].position.x;
        y = modelStatus.pose[2].position.y;
    }
    void pubAction_x_y(float vel_x ,float vel_y){
       actionVector.linear.x = vel_x;
       actionVector.linear.y = vel_y;
       pub_cmd_vel.publish(actionVector);
    }
    int takeAction(int action){
       float vel_x ;
       float vel_y ;
       int action_angle ;
       if(action == 0){
         std::cout<<"Action Forward"<<std::endl;
         vel_x = 0.2 ;
         vel_y = 0.0 ;
         action_angle = 0 ;
       }
       else if(action == 1){
         std::cout<<"Action left"<<std::endl;
         vel_x = 0.0 ;
         vel_y = 0.2 ;
         action_angle = 90 ;
       }
       else if(action == 2){
         std::cout<<"Action left Forward"<<std::endl;
         vel_x = 0.2 ;
         vel_y = 0.2 ;
         action_angle = 45 ;
       }
       else if(action == 3){
         std::cout<<"Action right"<<std::endl;
         vel_x = 0.0 ;
         vel_y = -0.2 ;
         action_angle = -90 ;
       }
       else if(action == 4){
         std::cout<<"Action right Forward"<<std::endl;
         vel_x = 0.2 ;
         vel_y = -0.2 ;
         action_angle = -45 ;
       }
       else {
         std::cout<<"Action wrong"<<std::endl;
         vel_x = 0.0 ;
         vel_y = 0.0 ;
         action_angle = 0 ;
       }
       pubAction_x_y(vel_x ,vel_y);
       return action_angle;
    }
    bool isCollided(auto new_state){
       float min_new_state = *std::min_element(std::begin(new_state), std::end(new_state));
       std::cout<<"min_new_statemin:"<<min_new_state<<std::endl;
       if(min_new_state <= 0.16){
         return true;
       }
       else{
         return false;
       }
    }
    bool isGoal(float x,float y){
       if((x>=goal_x)&&(y<=goal_y)){
         return true;
       }
       else{
         return false;
       }
    }
    
    void setReward(int action_angle ,auto prev_state ,auto new_state ,float x ,float y,bool &done,int &reward){
       std::vector<float> state_forward = {prev_state[22],prev_state[23],prev_state[0],prev_state[1],prev_state[2]};
       std::vector<float> state_f_left = {prev_state[2],prev_state[3],prev_state[4],prev_state[5]};
       std::vector<float> state_left = {prev_state[5],prev_state[6],prev_state[7],prev_state[8]};
       std::vector<float> state_right = {prev_state[17],prev_state[18],prev_state[19]};
       std::vector<float> state_f_right = {prev_state[19],prev_state[20],prev_state[21],prev_state[22]};
       float max,min;
       if(isCollided(new_state)){
          std::cout<<"Collided detected !"<<std::endl;
          done = true ;
          reward = -10 ;
          std::cout<<"Reward :"<<reward<<std::endl;
       }
       else if(isGoal(x,y)){
          std::cout<<"Goal !"<<std::endl;
          done = true ;
          reward = 100 ;
          std::cout<<"Reward :"<<reward<<std::endl;          
       }
       else{
          done = false;
          if(action_angle==0){
                std::cout<<"In action forward reward setting"<<std::endl;
                for(int i=0;i<state_forward.size();i++)
                std::cout<<"state_forward["<<i<<"]"<<state_forward[i]<<std::endl;
                max = *std::max_element(std::begin(state_forward), std::end(state_forward));
                min = *std::min_element(std::begin(state_forward), std::end(state_forward));
                if(max < 0.25)
                    reward = -2;
                else if(max < 0.35)
                    reward = -0.5;
                else if(max < 0.45)
                    reward = 0;
                else if(max < 0.55)
                    reward = 0.5;
                else
                    reward = 1;

                if(min > 0.45)
                    reward += 2;
                else if(min > 0.25)
                    reward += 0 ;
                else
                    reward += -2 ;
          }
          if(action_angle==90){
                std::cout<<"In action left reward setting"<<std::endl;
                for(int i=0;i<state_left.size();i++)
                std::cout<<"state_left["<<i<<"]"<<state_left[i]<<std::endl;
                max = *std::max_element(std::begin(state_left), std::end(state_left));
                min = *std::min_element(std::begin(state_left), std::end(state_left));
                if(max < 0.25)
                    reward = -2;
                else if(max < 0.35)
                    reward = -0.5;
                else if(max < 0.45)
                    reward = 0;
                else if(max < 0.55)
                    reward = 0.5;
                else
                    reward = 1;

                if(min > 0.45)
                    reward += 2;
                else if(min > 0.25)
                    reward += 0 ;
                else
                    reward += -2 ;
          }
          if(action_angle==45){
                std::cout<<"In action left forward reward setting"<<std::endl;
                for(int i=0;i<state_f_left.size();i++)
                std::cout<<"state_f_left["<<i<<"]"<<state_f_left[i]<<std::endl;
                max = *std::max_element(std::begin(state_f_left), std::end(state_f_left));
                min = *std::min_element(std::begin(state_f_left), std::end(state_f_left));
                if(max < 0.25)
                    reward = -2;
                else if(max < 0.35)
                    reward = -0.5;
                else if(max < 0.45)
                    reward = 0;
                else if(max < 0.55)
                    reward = 0.5;
                else
                    reward = 1;

                if(min > 0.45)
                    reward += 2;
                else if(min > 0.25)
                    reward += 0 ;
                else
                    reward += -2 ;
          }
          if(action_angle==-90){
                std::cout<<"In action right reward setting"<<std::endl;
                for(int i=0;i<state_right.size();i++)
                std::cout<<"state_right["<<i<<"]"<<state_right[i]<<std::endl;
                max = *std::max_element(std::begin(state_right), std::end(state_right));
                min = *std::min_element(std::begin(state_right), std::end(state_right));
                if(max < 0.25)
                    reward = -2;
                else if(max < 0.35)
                    reward = -0.5;
                else if(max < 0.45)
                    reward = 0;
                else if(max < 0.55)
                    reward = 0.5;
                else
                    reward = 1;

                if(min > 0.45)
                    reward += 2;
                else if(min > 0.25)
                    reward += 0 ;
                else
                    reward += -2 ;
          }
          if(action_angle==-45){
                std::cout<<"In action right forward reward setting"<<std::endl;
                for(int i=0;i<state_right.size();i++)
                std::cout<<"state_f_right["<<i<<"]"<<state_f_right[i]<<std::endl;
                max = *std::max_element(std::begin(state_f_right), std::end(state_f_right));
                min = *std::min_element(std::begin(state_f_right), std::end(state_f_right));
                if(max < 0.25)
                    reward = -2;
                else if(max < 0.35)
                    reward = -0.5;
                else if(max < 0.45)
                    reward = 0;
                else if(max < 0.55)
                    reward = 0.5;
                else
                    reward = 1;

                if(min > 0.45)
                    reward += 2;
                else if(min > 0.25)
                    reward += 0 ;
                else
                    reward += -2 ;
          }
       }
    }
    auto step(auto prev_state, int action,auto &new_state,bool &done,int &reward){
       int action_angle = takeAction(action);
       new_state = getLaserScan();
       std::cout<<robot_x<<","<<robot_y<<std::endl;
       prev_robot_x = robot_x;
       prev_robot_y = robot_y;
       getRobotStatus(robot_x,robot_y);
       std::cout<<robot_x<<","<<robot_y<<std::endl;
       setReward(action_angle,prev_state,new_state,robot_x,robot_y,done,reward);
    }
    auto reset(){
       pubAction_x_y(0,0);
       std_srvs::Empty srv;
       if (client.call(srv))
       {
         ROS_INFO("Reset DONE");
       }
       else
       {
         ROS_ERROR("Failed to call service reset_simulation");
       }
       sleep(2);
       return getLaserScan();
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "client_pg");
  ros::NodeHandle nh;
  bool done = false;
  int reward;
  int action;
  ENV env = ENV(&nh);
  sleep(2);
  ros::spinOnce();
  std::vector<float> state = env.reset();

  //for(int i=0;i<24;i++)
  //std::cout<i<<":"<<state[i]<<std::endl;
  

  /*TEST
  while(true){
    state = env.getLaserScan();
    float min_new_state = *std::min_element(std::begin(state), std::end(state));
    std::cout<<"min:"<<min_new_state<<std::endl;
    sleep(1);
    if(min_new_state<=0.16)
      break;
    ros::spinOnce();
    }
  */
  
  while(done == false){
    std::cout<<"action:"<<std::endl;
    std::cin>>action;
    env.step(state,action,state,done,reward);
    std::cout<<"Reward:"<<reward<<std::endl;
    ros::spinOnce();
  }
  
  //ros::spin();
  return 0;
}

