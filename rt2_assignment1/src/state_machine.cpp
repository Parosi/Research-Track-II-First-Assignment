#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "actionlib/client/simple_action_client.h"
#include "rt2_assignment1/GoToPointAction.h"

bool start = false;
bool previous_start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	previous_start = start;
        start = true;
    }
    else {
        previous_start = start;
    	start = false;
    }
    return true;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient<rt2_assignment1::GoToPointAction> ac("goToPoint", true);
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   // Wait for the action server to start
   ac.waitForServer(); // Will wait for infinite time 

   rt2_assignment1::GoToPointGoal goal;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start && !previous_start){
   		client_rp.call(rp);
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " <<goal.y << " theta = " <<goal.theta << std::endl;
        // Send the goal
        ac.sendGoal(goal);
        // Wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(0.1));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            std::cout << "Position reached" << std::endl;
            previous_start = false;
        }

        previous_start = start;
   	}

    else if (start && previous_start) {

        // Wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(0.1));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            std::cout << "\nPosition reached" << std::endl;
            previous_start = false;
        }
    } 

    else if (!start && previous_start) {
        // Cancel the goal
        ac.cancelGoal();
        std::cout << "\nRobot stopped" << std::endl;

        previous_start = start;
    }
   }
   return 0;
}
