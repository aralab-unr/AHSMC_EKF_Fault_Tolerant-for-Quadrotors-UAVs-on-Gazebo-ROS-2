#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Dense>
#include <fstream>

using namespace Eigen;

template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

class PID {
public:
    PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), previous_error_(0), integral_(0) {}

    double calculate(double setpoint, double pv, double dt) {
        double error = setpoint - pv;
        double Pout = kp_ * error;
        integral_ += error * dt;
        integral_ = clamp(integral_, -0.075, 0.075);
        double Iout = ki_ * integral_;
        double derivative = (error - previous_error_) / dt;
        double Dout = kd_ * derivative;
        double output = Pout + Iout + Dout;
        previous_error_ = error;
        return output;
    }

private:
    double kp_, ki_, kd_;
    double previous_error_, integral_;
};

class PIDXY {
public:
    PIDXY(double kpphi, double kiphi,double kdphi, double kptheta, double kitheta, double kdtheta,  double kpx, double kdx, double kpy, double kdy) 
        : kpphi_(kpphi), kiphi_(kiphi), kdphi_(kdphi),  kptheta_(kptheta), kitheta_(kitheta), kdtheta_(kdtheta),  kpx_(kpx), kdx_(kdx), kpy_(kpy), kdy_(kdy), 
          previous_error_x(0), previous_error_phi(0), previous_error_y(0), previous_error_theta(0), integralphi(0), integraltheta(0) {}

    std::pair<double, double> calculatexy(double setpointx, double setpointy, double pvx, double pvy, double pvphi, double pvtheta, double psi, double dt) {
        // Calculate error
        double errorx = setpointx - pvx;
        double errory = setpointy - pvy;
        double derivativex = (errorx - previous_error_x) / dt;
        double derivativey = (errory - previous_error_y) / dt;
        previous_error_x = errorx;
        previous_error_y = errory;

        // Proportional term
        double thetade = kpx_ * errorx + kdx_ * derivativex;
        double phide = -kpy_ * errory - kdy_ * derivativey;
        double phir = phide * cos(psi) + thetade * sin(psi);
        double thetar = -phide * sin(psi) + thetade * cos(psi);

        thetar = clamp(thetar, -0.1, 0.1);
        phir = clamp(phir, -0.1, 0.1);

        double errorphi = phir - pvphi;
        double errortheta = thetar - pvtheta;
        double derivativephi = (errorphi - previous_error_phi) / dt;
        double derivativetheta = (errortheta - previous_error_theta) / dt;
        integralphi += errorphi * dt;
        integralphi = clamp(integralphi, -0.075, 0.075);
        integraltheta += errortheta * dt;
        integraltheta = clamp(integraltheta, -0.075, 0.075);
        previous_error_phi = errorphi;
        previous_error_theta = errortheta;

        double U2 = kpphi_ * errorphi + kdphi_ * derivativephi + kiphi_*integralphi;
        double U3 = kptheta_ * errortheta + kdtheta_ * derivativetheta + kitheta_ * integraltheta;
        U2 = clamp(U2, -2.0, 2.0);
        U3 = clamp(U3, -2.0, 2.0);

        return {U2, U3};
    }

private:
    double kpx_, kdx_, kpy_, kdy_;
    double kdphi_, kpphi_, kdtheta_, kptheta_, kiphi_, kitheta_;
    double previous_error_x, previous_error_phi;
    double integralphi, integraltheta;
    double previous_error_y, previous_error_theta;
};


class PIDUAV : public rclcpp::Node {
public:
    PIDUAV(): Node("pid_uav"), controlz_(8.0, 0, 5.75), controlpsi_(3.25, 0.5, 1.75), controlxy_(1.05, 0.5275, 1.0, 1.05, 0.5275, 1.0, 0.05, 0.1, 0.05, 0.1), EKF_state_(12)
    {
        // Publisher for propeller velocities
        propvel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/prop_vel", 10);
        control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/controllaw", 10);
        state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/ekf_state", 10, std::bind(&PIDUAV::stateCallback, this, std::placeholders::_1));
        last_time_ = this->now();
    }
    void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 12) {
        RCLCPP_WARN(this->get_logger(), "Received unexpected optimal state size.");
        return;
        }
        for (size_t i = 0; i < 12; ++i) {
            EKF_state_(i) = msg->data[i];
        }
        double x = EKF_state_(0);
        double y = EKF_state_(1);
        double z = EKF_state_(2);
        double phi = EKF_state_(3);
        double theta = EKF_state_(4);
        double psi = EKF_state_(5);
        double xd = EKF_state_(6);
        double yd = EKF_state_(7);
        double zd = EKF_state_(8);
        double phid = EKF_state_(9);
        double thetad = EKF_state_(10);
        double psid = EKF_state_(11);

        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();

        last_time_ = current_time;
        double xdes = 5, ydes = 5, zdes = 5.0, psides = 0;

        double U1 = clamp(controlz_.calculate(zdes, z, dt) + 14.715, 11.5, 18.5);
        double U4 = clamp(controlpsi_.calculate(psides, psi, dt), -0.25, 0.25);
        auto [U2, U3] = controlxy_.calculatexy(xdes, ydes, x, y, phi, theta, psi, dt);

        double kt = 0.00025, kd = 0.000075, l = 0.175;
        double w12, w22, w32, w42;

        w12 = (U1 * kd * l - U2 * kd - U3 * kd + U4 * kt * l) / (4 * kd * kt * l);
        w22 = (U1 * kd * l + U2 * kd - U3 * kd - U4 * kt * l) / (4 * kd * kt * l);
        w32 = (U1 * kd * l - U2 * kd + U3 * kd - U4 * kt * l) / (4 * kd * kt * l);
        w42 = (U1 * kd * l + U2 * kd + U3 * kd + U4 * kt * l) / (4 * kd * kt * l);

        // Ensure non-negative wheel speeds
        w12 = std::max(0.0, w12);
        w22 = std::max(0.0, w22);
        w32 = std::max(0.0, w32);
        w42 = std::max(0.0, w42);

        double w1 = -std::sqrt(w12);
        double w2 = std::sqrt(w22);
        double w3 = std::sqrt(w32);
        double w4 = -std::sqrt(w42);
        // Publish propeller velocities
        std_msgs::msg::Float64MultiArray prop_vel_msg;
        prop_vel_msg.data = {w1, w2, w3, w4};
        propvel_pub_->publish(prop_vel_msg);

        // Publish control law
        std_msgs::msg::Float64MultiArray control_msg;
        control_msg.data = {U1, U2, U3, U4};
        control_pub_->publish(control_msg);

        //auto node = rclcpp::Node::make_shared("time_logger");
        //rclcpp::Clock::SharedPtr clock = node->get_clock();

        //auto now = clock->now();
        //auto elapsed_time = now.nanoseconds(); // ROS time in nanoseconds

        //auto elapsed_time_ms = elapsed_time / 1'000'000; 

        //std::ofstream outfile1("datapid/datapidxyz.txt", std::ios_base::app);
        //if (outfile1.is_open()) {
        //    outfile1 << "Time: " << elapsed_time_ms << ", x: " << x << ", y: " << y << ", z: " << z << ", vx: " << xd << ", vy: " << yd << ", vz: " << zd <<"\n";
        //    outfile1.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}

        //std::ofstream outfile2("datapid/datapidrpy.txt", std::ios_base::app);
        //if (outfile2.is_open()) {
        //    outfile2 << "Time: " << elapsed_time_ms << ", Yaw desi: " << psides << ", Roll: " << phi << ", Pitch: " << theta << ", Yaw: " << psi << ", Rolld: " << phid << ", Pitchd: " << thetad << ", Yawd: " << psid << "\n";
        //    outfile2.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}

        //std::ofstream outfile3("datapid/datapidcontrol.txt", std::ios_base::app);
        //if (outfile3.is_open()) {
        //   outfile3 << "Time: " << elapsed_time_ms << ", U1: " << U1 << ", U2: " << U2 << ", U3: " << U3 << ", U4: " << U4 << ", w1: " << w1 << ", w2: " << w2 << ", w3: " << w3 << ", w4: " << w4 << "\n";
        //    outfile3.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr propvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;

    PID controlz_;
    PID controlpsi_;
    PIDXY controlxy_;
    rclcpp::Time last_time_;
    double prephir, prethetar, prepsir;
    Eigen::VectorXd EKF_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<PIDUAV>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}