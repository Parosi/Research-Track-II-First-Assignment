'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
This node implements a state machine which permits to control the 
various states of the robot.

It creates a ros service for receiving the commands from the user.
It also implements a service client which is used to get a random 
position and an action client which is used to move the robot.
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"

using namespace std::chrono_literals;
using Command = rt2_assignment1::srv::Command;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using Position = rt2_assignment1::srv::Position;

//Placeholders for the three arguments of the service's callback
using std::placeholders::_1;
using std::placeholders:: _2;
using std::placeholders:: _3;

namespace rt2_assignment1{

  class StateMachine : public rclcpp::Node
  {
    public:
      StateMachine(const rclcpp::NodeOptions & options)
      : Node("state_machine", options), count_(0)
      {
        // Initialize bools
        start_ = false;
        rp_waiting_ = false;
        p_waiting_ = false;

        // Create the server
        service_ = this -> create_service<Command>("user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
        std::cout << "User interface service started" << std::endl;
        // Create the first client
        client_rp_ = this -> create_client<RandomPosition>("/position_server");
        // Wait the random position service
        while (!client_rp_ -> wait_for_service(std::chrono::seconds(1))){
            if (!rclcpp::ok()){
                RCLCPP_ERROR(this -> get_logger(), 
                  "Random position client interrupted while waiting for service to appear.");
                  exit(0);
            }
        }
        std::cout << "Random position client connected to the server" << std::endl;
        // Create the second client
        client_p_ = this -> create_client<Position>("/go_to_point");
        // Wait the position service
        while (!client_p_ -> wait_for_service(std::chrono::seconds(1))){
            if (!rclcpp::ok()){
                RCLCPP_ERROR(this -> get_logger(), 
                  "Go to point client interrupted while waiting for service to appear.");
                  exit(0);
            }
        }
        std::cout << "Position client connected to the server" << std::endl;

        // Instantiate all messages
        this -> rp_request_ = std::make_shared<RandomPosition::Request>();
        this -> p_request_ = std::make_shared<Position::Request>();
        this -> rp_response_ = std::make_shared<RandomPosition::Response>();
        this -> p_response_ = std::make_shared<Position::Response>();

        // Set the random position request
        rp_request_ -> x_max = 5.0;
        rp_request_ -> x_min = -5.0;
        rp_request_-> y_max = 5.0;
        rp_request_ -> y_min = -5.0;

        // Start the timer
        timer_ = this->create_wall_timer(500ms, std::bind(&StateMachine::timer_callback, this));

      }

    private:

      void user_interface(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<Command::Request> request,
        const std::shared_ptr<Command::Response> response)
      {

        (void)request_header;

        if (request -> command == "start"){
    	  this -> start_ = true;
        }
        else {
    	  this -> start_ = false;
        }

        // Set the response
        response -> ok = true;
        
      }

      void timer_callback(){

   	    if (start_ && !rp_waiting_ && !p_waiting_){

          // Call the pose service
          rp_result_future_ = client_rp_ -> async_send_request(rp_request_);
          rp_waiting_ = true;

   	    }

        else if (rp_waiting_) {
          if (rp_result_future_.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
            // Get response
            rp_response_ = rp_result_future_.get();
            p_request_ -> x = rp_response_ -> x;
            p_request_ -> y = rp_response_ -> y;
            p_request_ -> theta = rp_response_ -> theta;
            std::cout << "\nGoing to the position: x= " << p_request_ -> x << " y= " <<p_request_ -> y << " theta = " << p_request_ -> theta << std::endl;
            // Call the go to point service
            p_result_future_ = client_p_ -> async_send_request(p_request_);
            p_waiting_ = true;
            rp_waiting_ = false;
          }
        }

        else if (p_waiting_) {
          if (p_result_future_.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
            // Get response
            p_response_ = p_result_future_.get();
            std::cout << "Position reached" << std::endl;
            p_waiting_ = false;
          }
        }
      }

      rclcpp::Service<Command>::SharedPtr service_;
      rclcpp::Client<RandomPosition>::SharedPtr client_rp_;
      rclcpp::Client<Position>::SharedPtr client_p_;
      std::shared_ptr<RandomPosition::Request> rp_request_;
      std::shared_ptr<Position::Request> p_request_;
      std::shared_ptr<RandomPosition::Response> rp_response_;
      std::shared_ptr<Position::Response> p_response_;
      rclcpp::TimerBase::SharedPtr timer_;
      bool start_;
      bool rp_waiting_;
      bool p_waiting_;
      size_t count_;
      std::shared_future<RandomPosition::Response::SharedPtr> rp_result_future_;
      std::shared_future<Position::Response::SharedPtr> p_result_future_;
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)