#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Dense>
#include <fstream>


using namespace Eigen;

// Corrected clamp function
template <typename T>
T clamp(T value, T min, T max) {
    return std::max(min, std::min(max, value));
}

class PIDXY {
public:
    PIDXY(double kpx, double kdx, double kpy, double kdy) 
        : kpx_(kpx), kdx_(kdx), kpy_(kpy), kdy_(kdy), 
          previous_error_x(0), previous_error_y(0) {}

    std::pair<double, double> calculatexy(double setpointx, double setpointy, double pvx, double pvy, double pvphi, double pvtheta, double dt) {
        // Calculate error
        double errorx = setpointx - pvx;
        double errory = setpointy - pvy;
        double derivativex = (dt > 1e-5) ? (errorx - previous_error_x) / dt : 0; // Avoid dividing by zero or very small dt
        double derivativey = (dt > 1e-5) ? (errory - previous_error_y) / dt : 0;

        previous_error_x = errorx;
        previous_error_y = errory;

        // Proportional term for roll and pitch
        double thetar = kpx_ * errorx + kdx_ * derivativex;
        double phir = -kpy_ * errory - kdy_ * derivativey;

        return {phir, thetar};
    }

private:
    double kpx_, kdx_, kpy_, kdy_;
    double previous_error_x, previous_error_y;
};


class HSMCUAV : public rclcpp::Node {
public:
    HSMCUAV(): Node("hsmc_uav"), prephir(0), prethetar(0), prepsir(0), controlxy_(0.05, 0.1, 0.05, 0.1), EKF_state_(12)
    {
        // Publisher for propeller velocities
        propvel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/prop_vel", 10);
        control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/controllaw", 10);
        state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/ekf_state", 10, std::bind(&HSMCUAV::stateCallback, this, std::placeholders::_1));
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
        double xr = 5, yr = 5, zr = 5.0, psir = 0;
        double xrd = 0, yrd = 0, zrd = 0;
        double xrdd = 0, yrdd = 0, zrdd = 0;
        double ixx = 0.0785, iyy = 0.0785, izz = 0.105;

        auto [phir, thetar] = controlxy_.calculatexy(xr, yr, x, y, phi, theta, dt);
        double phird = (phir - prephir) / dt;
        double thetard = (thetar - prethetar) / dt;
        double psird = (psir - prepsir) / dt;

        prephir = phir;
        prethetar = thetar;
        prepsir = psir;
        
        double cphi_ctrl = 3.25;
        double ctheta_ctrl = 3.25;
        double cpsi_ctrl = 1.75;
        
        double Kx = 3.5;
        double Ky = 3.5;
        double Kz = 1.25;

        double fphi = psid * thetad * (iyy - izz) / ixx;
        double ftheta = psid * phid * (izz - ixx) / iyy;
        double fpsi = phid * thetad * (ixx - iyy) / izz;

        double bphi = 1 / ixx;
        double btheta = 1 / iyy;
        double bpsi = 1 / izz;

        double Kphi = 0.75;
        double Ktheta = 0.75;
        double Kpsi = 0.25;

        double scphi = cphi_ctrl * (phi - phir) + (phid - phird);
        double sctheta = ctheta_ctrl * (theta - thetar) + (thetad - thetard);
        double scpsi = cpsi_ctrl * (psi - psir) + (psid - psird);

        double satphi = clamp(scphi, -0.1, 0.1);
        double sattheta = clamp(sctheta, -0.1, 0.1);
        double satpsi = clamp(scpsi, -0.1, 0.1);

        double U2s = (-Ky * cphi_ctrl * phi - (cphi_ctrl + Ky) * phid + Ky * cphi_ctrl * phir + (cphi_ctrl + Ky) * phird - fphi - Kphi * satphi) / bphi;
        double U3s = (-Kx * ctheta_ctrl * theta - (ctheta_ctrl + Kx) * thetad + Kx * ctheta_ctrl * thetar + (ctheta_ctrl + Kx) * thetard - ftheta - Ktheta * sattheta) / btheta;
        double U4 = (-Kz * cpsi_ctrl * psi - (cpsi_ctrl + Kz) * psid + Kz * cpsi_ctrl * psir + (cpsi_ctrl + Kz) * psird - fpsi - Kpsi * satpsi) / bpsi;

        double U2 = clamp(U2s, -0.2, 0.2);
        double U3 = clamp(U3s, -0.2, 0.2);

        double m = 1.5;
        double g = 9.81;
        double Kdx= 0.0000267;
        double Kdy= 0.0000267;
        double Kdz= 0.0000625;
        double cx = 0.015;
        double cy = 0.015;
        double cz = 0.175;
        
        double lam1 = 0.05;
        double lam2 = 0.05;
        double Ka = 10.75;
        double eta = 0.25;

        double cphi = cos(phi), sphi = sin(phi);
        double ctheta = cos(theta), stheta = sin(theta);
        double cpsi = cos(psi), spsi = sin(psi);

        double fx = -Kdx * xd / m;
        double fy = -Kdy * yd / m;
        double fz = (-Kdz * zd - m * g) / m;
        double bx = 1 / m * (spsi * sphi + cpsi * stheta * cphi);
        double by = 1 / m * (spsi * stheta * cphi - cpsi * sphi);
        double bz = 1 / m * (ctheta * cphi);

        double sx = cx * (xr - x) + (xrd - xd);
        double sy = cy * (yr - y) + (yrd - yd);
        double sz = cz * (zr - z) + (zrd - zd);

        double ueqx = (cx * (xrd - xd) + xrdd - fx) / bx;
        double ueqy = (cy * (yrd - yd) + yrdd - fy) / by;
        double ueqz = (cz * (zrd - zd) + zrdd - fz) / bz;

        double s3 = lam2 * lam1 * sx + lam2 * sy + sz;
        double sats3 = clamp(s3, -0.1, 0.1);

        double usw = -(lam2 * lam1 * bx * (ueqy + ueqz) + lam2 * by * (ueqx + ueqz) + bz * (ueqx + ueqy) - Ka * s3 - eta * sats3) / (lam2 * lam1 * bx + lam2 * by + bz);
        double Uz = ueqx + ueqy + ueqz + usw;

        double U1 = clamp(Uz, 11.5, 18.5);

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

    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr propvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;

    PIDXY controlxy_;
    rclcpp::Time last_time_;
    double prephir, prethetar, prepsir;
    Eigen::VectorXd EKF_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<HSMCUAV>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}