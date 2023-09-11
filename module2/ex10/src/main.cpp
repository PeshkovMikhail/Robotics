#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <ncurses.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using namespace geometry_msgs;

void fill_zero_vector3(msg::Vector3& vector) {
	vector.set__x(0);
	vector.set__y(0);
	vector.set__z(0);
}

class TurtleController : public rclcpp::Node
{
public:
	TurtleController()
	: Node("turtle_controller")
	{
		publisher_ = this->create_publisher<msg::Twist>("/turtle1/cmd_vel", 10);
		timer_ = this->create_wall_timer(
				20ms, std::bind(&TurtleController::timer_callback, this));
	}
private:
	void timer_callback() {
		auto message = msg::Twist();
		auto linear = msg::Vector3();
		fill_zero_vector3(linear);
		auto angular = msg::Vector3();
		fill_zero_vector3(angular);

		int ch = getch();
		switch(ch){
			case KEY_UP:
				std::cout << "FORWARD\r\n";
				linear.set__x(1);
				break;
			case KEY_DOWN:
				std::cout << "BACKWARD\r\n";
				linear.set__x(-1);
				break;
			case KEY_LEFT:
				std::cout << "TURN LEFT\r\n";
				angular.set__z(1.5);
				break;
			case KEY_RIGHT:
				std::cout << "TURN RIGHT\r\n";
				angular.set__z(-1.5);
				break;
			case 27:
				std::cout << "todo exit\r\n";
				refresh();
				endwin();
				rclcpp::shutdown();
				break;
			default:
				break;
		}

		message.set__angular(angular);
		message.set__linear(linear);
		publisher_->publish(message);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
	initscr();
	raw();
	keypad(stdscr, TRUE);
	noecho();

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TurtleController>());
	rclcpp::shutdown();
	return 0;
}
