#ifndef MPC_INCLUDED
#define MPC_INCLUDED

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <array>
#include <math.h>


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <lgsvl_msgs/CanBusData.h>
#include <tf/tf.h>
#include <tf/LinearMath/Scalar.h>
#include <autoware_msgs/VehicleCmd.h>
#include <geometry_msgs/Twist.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>




using namespace std;

using CppAD::AD;
using namespace Eigen;

class point{
    public:
        float x;
        float y;
        float z;
};


class MPC{
 public:

    size_t N = 10;
    double dt = 0.1;


    size_t x_start = 0;
    size_t y_start = x_start + N;
    size_t psi_start = y_start + N;
    size_t v_start = psi_start + N;
    size_t cte_start = v_start + N;
    size_t epsi_start = cte_start + N;
    size_t delta_start = epsi_start + N;
    size_t a_start = delta_start + N - 1;
    
    // This is the length from front to CoG that has a similar radius.
    const double Lf = 1;

    // Set desired speed for the cost function
    const double ref_v = 2;  

    //Constraints

    float steer_upperBound=1.0 *  6.2831853071796;
    float steer_lowerBound = -1.0 * 6.2831853071796;
    float acc_upperBound = 1.0;
    float acc_lowerBound = -1.0;
  
    //Future States
    double pred_px = 0;
    double pred_py = 0;
    double pred_psi=0;
    double pred_v = 0;
    double pred_cte = 0;
    double pred_epsi = 0;

    float acc_Longi;
    float acc_lateral;

    

  
    MPC();

    virtual ~MPC();
    
    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    
    //Predicts Future State of the car according to Kinematic Model.
    void predict_FutureState(float velocity_actual, float delta, float cte,float epsi, float acc_Longitudunal);




};




class Car:public MPC{
    private:
        std::string method; // pid or pp


        float dt = 01.;


        float velocityLimit; //If car moves, it will use this velocity
        float steer;
        float throttle;


        tfScalar roll, pitch, yaw; //from odometry
        float x_global;
        float y_global;
        float velocity_actual;
        float steering_angle;
       


        ros::NodeHandle* n;
        //Subscribers
        ros::Subscriber waypointLeftSub;
        ros::Subscriber odomSub;
        ros::Subscriber imuSub;
        ros::Subscriber canSub;
        
        //Publisher
        ros::Publisher cmdPub;
        
        std::vector< point > waypointsLeft;
        std::vector< point > waypointsRight;

    public:
        //Constructor of Car class
        Car (int argc , char **argv);


        void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void imuCallback(const sensor_msgs::Imu& imuMsg);
        void canBusCallback(const lgsvl_msgs::CanBusData& CanMsg);

        
        void sendCommandToSim(float steer, float vel);

        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order);
        float polyeval(Eigen::VectorXd coeffs, float x); 
        void modelPredictiveController();
        
};

class FG_eval : public MPC{
 public:
  
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs;}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars);


};


#endif