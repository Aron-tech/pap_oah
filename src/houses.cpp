#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "math.h"
#include <iomanip>
#include <ctime>
#include <iostream>

using namespace std::chrono_literals;

class TurtleDrawer : public rclcpp::Node
{
public:
    TurtleDrawer() : Node("turtle_drawer")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    }

    void draw_line_repeated(int repeat_count, float distance, float speed)
    {
        for (int i = 0; i < repeat_count; ++i)
        {
            draw_line(distance, speed);
        }
    }

    void turn_45_degrees_repeated(int repeat_count)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = M_PI / 4; // 45 fok radiánban
        for (int i = 0; i < repeat_count; ++i)
        {
            cmd_pub_->publish(msg);
            rclcpp::sleep_for(1s);
        }
        msg.angular.z = 0;
        cmd_pub_->publish(msg);
    }

    void draw_house(float start_x, float start_y)
    {
        // Kezdőpozíció
        set_pen(false, 0, 0, 0);
        move_turtle(start_x, start_y, 0);

        //Falak
        for(int i = 0; i < 3; i++){
            set_pen(true, 255, 255, 255); //Fehér legyen a toll
            draw_line_repeated(2, 1.0f, 2.0f);
            turn_45_degrees_repeated(2);
           
            draw_line_repeated(2, 1.0f, 2.0f);
            turn_45_degrees_repeated(2);
        }

        //Tető
        set_pen(true, 255, 0, 0); //Piros legyen a toll
        turn_45_degrees_repeated(7);
        draw_line_repeated(1, 1.5f, 1.5f);
        turn_45_degrees_repeated(2);
        draw_line_repeated(1, 1.5f, 1.5f);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;

    void move_turtle(float x, float y, float theta)
    {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto result = client_->async_send_request(request);
    }

    void set_pen(bool enable, int red, int green, int blue)
    {
        auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
        pen_request->r = red;
        pen_request->g = green;
        pen_request->b = blue;
        pen_request->width = enable ? 3 : 0;
        pen_request->off = !enable;
        auto result = pen_client_->async_send_request(pen_request);
    }

    void draw_line(float distance, float speed)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = speed;
        cmd_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(distance / speed * 1000)));
        msg.linear.x = 0;
        cmd_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    int house_count = 1;

    if (argc > 1) {
        try {
            house_count = std::stoi(argv[1]);
            if (house_count < 1 || house_count > 3) {
                std::cerr << "A házak számának 1 és 3 között kell lennie. Alapértelmezett érték (1) használata." << std::endl;
                house_count = 1;
            }
        } catch (const std::invalid_argument& e) {
            std::cerr << "Érvénytelen bemenet. Alapértelmezett érték (1) használata." << std::endl;
            house_count = 1;
        }
    }

    auto drawer = std::make_shared<TurtleDrawer>();
    rclcpp::WallRate loop_rate(1s);

    float start_x = 2.0f; // Kezdő X
    float start_y = 5.5f; // Kezdő Y
    float house_spacing = 4.0f; // Házak közötti távolság

    for(int i = 0; i < house_count; i++) {
        drawer->draw_house(start_x + i * house_spacing, start_y);
    }

    rclcpp::shutdown();
    return 0;
}