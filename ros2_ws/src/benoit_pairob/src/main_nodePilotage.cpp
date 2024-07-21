#include "benoit_pairob/main_nodePilotage.hpp"

PilotageNode::PilotageNode() : Node("pilotage_node") {
    init_interfaces();
    init_parameters();
    timer_ = this->create_wall_timer(loop_dt_, std::bind(&PilotageNode::timer_callback, this));
}

PilotageNode::~PilotageNode() {}

void PilotageNode::init_interfaces(){
    publisher_command_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel", 10);
    subscriber_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&PilotageNode::set_target_teleop, this, std::placeholders::_1));
    subscriber_heading_ = this->create_subscription<sensor_msgs::msg::Imu>("/demo/imu", 10, std::bind(&PilotageNode::set_theta, this, std::placeholders::_1));
    subscriber_target_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/xy_closest_ball", 10, std::bind(&PilotageNode::set_target, this, std::placeholders::_1));
    subscriber_position_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/xy_robot", 10, std::bind(&PilotageNode::set_x, this, std::placeholders::_1));
}

void PilotageNode::init_parameters() {
    loop_dt_ = 40ms;
    x_ << 0, 0, 0;
    target_ << 0, 0;
    k = 0.5;
    u1_ = 0;
    u2_ = 0;
    norm = 150;
    teleop = false;
    button_pressed = false;
    v_ = 2.;
    fsm_ = 0;
    avance_ = false;
    ball_same_side_ = false;
}

void PilotageNode::set_theta(sensor_msgs::msg::Imu pose) {
    double theta = tf2::getYaw(pose.orientation);
    x_ << x_(0), x_(1), theta;
}

void PilotageNode::set_x(geometry_msgs::msg::PoseStamped pose) {
    double x = -pose.pose.position.y;
    double y = -pose.pose.position.x;
    x_ << x, y, x_(2);
}

void PilotageNode::set_target(geometry_msgs::msg::PoseStamped pose) {
    double x = -pose.pose.position.y;
    double y = -pose.pose.position.x;
    target_ << x, y;
}

void PilotageNode::set_target_teleop(sensor_msgs::msg::Joy joy) {
    double x = joy.axes[3];
    double y = joy.axes[2];
    if (joy.buttons[0] == 1 && !button_pressed)
    {
        teleop = !teleop;
        button_pressed = true;
    } else if (joy.buttons[0] == 0 && button_pressed)
    {
        button_pressed = false;
    }
    target_teleop << x, y;
}

void PilotageNode::timer_callback(){
    auto message = geometry_msgs::msg::Twist();

    if (teleop)
    {
        message.linear.x = target_teleop(0);
        if (target_teleop(0) < 0)
        {
            message.angular.z = -target_teleop(1);
        } else {
            message.angular.z = target_teleop(1);
        }
    } else {
        control();
        message.linear.x = u2_;
        message.angular.z = u1_;
    }
    // RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "commande : %f %f", u1_, u2_);
    publisher_command_->publish(message);
}

void PilotageNode::control() {
    ball_on_the_same_side();
    planning();
    double e = atan2(target_planned(1)-x_(1), target_planned(0)-x_(0))-x_(2);

    RCLCPP_INFO(this->get_logger(), "x = %f, y = %f, theta = %f", x_(0), x_(1), x_(2));
    RCLCPP_INFO(this->get_logger(), "target_x = %f, target_y = %f", target_planned(0), target_planned(1));
    RCLCPP_INFO(this->get_logger(), "u1 = %f, u2 = %f", u1_, u2_);
    RCLCPP_INFO(this->get_logger(), "fsm = %d", fsm_);
    RCLCPP_INFO(this->get_logger(), "avance = %d", avance_);
    RCLCPP_INFO(this->get_logger(), "e = %f", e);
    RCLCPP_INFO(this->get_logger(), "ball_same_side = %d", ball_same_side_);
    RCLCPP_INFO(this->get_logger(), "norm = %f", norm);

    e = 2*atan(tan(e/2));

    if (e > M_PI/4)
    {
        u1_ = M_PI/5;
    } else if (e < -M_PI/4)
    {
        u1_ = -M_PI/5;
    } else {
        u1_ = k*e;
    }
    //if (-0.05 <= e <=0.05)
    //if (((e<=0.05 && e>=-0.05) || abs(e - M_PI)<=0.05) e<=0.05 && e>=-0.05)
    if (e<=0.05 && e>=-0.05 && avance_)
    {
        u1_ = 0;
        u2_ = 1;
    } else {
        u2_ = 0;
    }

    // RCLCPP_INFO(this->get_logger(), "avance = %d e = %f", avance_, std::fmod(e, M_PI));
}


void PilotageNode::ball_on_the_same_side() {
    if (target_(0) == 0 && target_(1) == 0) ball_same_side_ = false;
    else if (-target_(1) > img_w/2 && -x_(1) < img_w/2)  {
        // CHANGER DE COTE --> regarder en haut ou en bas puis passage
        ball_same_side_ = false;
        // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "changement de cote");
    }
    else if (-target_(1) < img_w/2 && -x_(1) > img_w/2) {
        // CHANGER DE COTE --> regarder en haut ou en bas
        ball_same_side_ = false;
        // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "changement de cote");
    }
    else ball_same_side_ = true;
}

void PilotageNode::planning() {

    // check position balle
    bool ball_presence = target_(0) != 0 || target_(1) != 0;

    Matrix<double, 2, 1> target_planned_r;
    Matrix<double, 2, 2> R{{cos(x_(2)), -sin(x_(2))}, {sin(x_(2)), cos(x_(2))}};

    bool condition_mur = (-target_(1) > zone_E[0] && -target_(1) < filet_1[0] - 1.5*coef_x && -target_(0) < zone_F[1] && -target_(0) > zone_D[1])
                         || (-target_(1) > filet_1[0] + 1.5*coef_x && -target_(1) < zone_A[0] && -target_(0) < zone_F[1] && -target_(0) > zone_D[1]);
    if (fsm_ == 0 && ball_same_side_) {
        avance_ = true;
        if (condition_mur) { // près du mur : passage par un point intermédiaire
            fsm_ = 2;
        }
        else { // au centre : on va vers la balle
            fsm_ = 1;
            target_planned = target_;
        }
    }
    else if (!ball_same_side_ && fsm_ != 3) {    // todo probleme
        avance_ = false;
        fsm_ = 0;
    }

    else if (fsm_ == 1 && ball_same_side_) {
        //RCLCPP_INFO(this->get_logger(), "target_planned_r = %f, %f", target_planned_r(0), target_planned_r(1));
        norm = std::sqrt(std::pow(target_planned(0) - x_(0), 2) + std::pow(target_planned(1) - x_(1), 2));
        if (norm < 15) fsm_ = 3;    // go vers la zone
        else target_planned = target_;
    }

    else if (fsm_ == 2 && ball_same_side_) {
        double eps_x = coef_x * 1.5; // marge
        double eps_y = coef_y * 1.5;
        double dist = eps_x;
        Matrix<double, 2, 1> p; // point de la trajectoire
        if (1280 + target_(1) < eps_x || -target_(1) < eps_x) {
            // mur du bas ou du haut
            p << target_(0), target_(1) - dist;
        }
        else if (720 + target_(0) < eps_y || -target_(0) < eps_y) {
            // mur de gauche ou de droite
            p << target_(0) - dist, target_(1);
        }

        // vérifier qu'on est proche du point intermédiaire
        norm = std::sqrt(std::pow(p(0) - x_(0), 2) + std::pow(p(1) - x_(1), 2));
        if (norm > 20) target_planned = p;
        else {
            fsm_ = 1;  // on va vers la balle
            // target_planned = target_;
        }
    }

    else if (fsm_ == 3) {
        // pour l'instant on reste du coté des points C et B
        // on veut rejoindre le point entre C et B
        // on vérifie si on est au dessus de la porte
        if (-x_(0) > zone_C[1]) {
            // on va vers au-dessus
            //target_planned = {x_(0), -zone_B[0] + 30};
            target_planned = {x_(0), -zone_B[0] + 30};
        }
        else if (-x_(1) > zone_B[0]) {
            // on va vers la gauche
            //target_planned = {-zone_C[1] + 30, x_(1)};
            target_planned = {-zone_C[1] + 30, x_(1)};
        }
        else {
            // on va entre les portes
            //target_planned = {-zone_B[1] + zone_C[1], -zone_B[0] + zone_C[0]};
            target_planned = {-zone_C[1] + (-zone_B[1] + zone_C[1])/2, -zone_C[0] + (-zone_B[0] + zone_C[0])/2};
            norm = std::sqrt(std::pow(target_planned(0) - x_(0), 2) + std::pow(target_planned(1) - x_(1), 2));
            if (norm < 10) fsm_ = 0;
        }
    }
    // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "fsm : %d ; norm : %f", fsm_, norm);
}

/****************************************
 
    MAIN

*****************************************/

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PilotageNode>());
    rclcpp::shutdown();
    return 0;
}