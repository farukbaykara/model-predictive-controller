

#include <mpc.h>

Car::Car(int argc, char **argv)
{

    ros::init(argc, argv, "mpc_controller");
    n = new ros::NodeHandle("~");

    waypointLeftSub = n->subscribe("/waypoints", 1, &Car::waypointCallback, this);

    odomSub = n->subscribe("/odom", 1, &Car::odomCallback, this);

    imuSub = n->subscribe("/imu_raw", 1, &Car::imuCallback, this);

    canSub = n->subscribe("/canbus", 1, &Car::canBusCallback, this);

    cmdPub = n->advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);

    ros::spin();
}

void Car::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{

    x_global = -odomMsg->pose.pose.position.x; // multiplied by -1 cause of reverse coordinates
    y_global = -odomMsg->pose.pose.position.y;

    tf::Quaternion q(
        odomMsg->pose.pose.orientation.x,
        odomMsg->pose.pose.orientation.y,
        odomMsg->pose.pose.orientation.z,
        odomMsg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    tf::Matrix3x3 m2;
    m2.setEulerYPR(3.14, 0, 0);
    m = m2 * m;

    m.getRPY(roll, pitch, yaw); // gets yaw,pitch ,roll from quaternions.
    yaw = -yaw;
    velocity_actual = odomMsg->twist.twist.linear.x;
}

void Car::imuCallback(const sensor_msgs::Imu &imuMsg)
{

    acc_Longi = imuMsg.linear_acceleration.x;
    acc_lateral = imuMsg.linear_acceleration.y;
}

void Car::canBusCallback(const lgsvl_msgs::CanBusData &CanMsg)
{

    steering_angle = CanMsg.steer_pct * 6.2831853071796; // int is the max steering angle
}

void Car::waypointCallback(const nav_msgs::PathConstPtr &waypointMsg)
{

    std::vector<geometry_msgs::Point> temp_path;

    temp_path.resize(waypointMsg->poses.size());

    for (long unsigned int i = 0; i < temp_path.size(); i++)
    {
        temp_path[i].x = waypointMsg->poses[i].pose.position.x;
        temp_path[i].y = waypointMsg->poses[i].pose.position.y;
        temp_path[i].z = waypointMsg->poses[i].pose.position.z;
    }

    modelPredictiveController(temp_path);
}

void Car::sendCommandToSim(float steer, float vel)
{

    autoware_msgs::VehicleCmd cmdMsg;
    cmdMsg.twist_cmd.twist.angular.z = steer;
    cmdMsg.twist_cmd.twist.linear.x = vel;

    cmdPub.publish(cmdMsg);
}

float Car::clip(float n, float lower, float upper)
{
    return std::max(lower, std::min(n, upper));
}

Eigen::VectorXd Car::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

float Car::polyeval(Eigen::VectorXd coeffs, float x)
{
    float result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

void Car::modelPredictiveController(const std::vector<geometry_msgs::Point> &t_path)
{

    double t_velocity = 0;

    MPC mpc;

    Eigen::VectorXd t_waypoints_x(t_path.size());
    Eigen::VectorXd t_waypoints_y(t_path.size());

    for (int i = 0; i < t_path.size(); i++)
    {
        t_waypoints_x[i] = t_path[t_path.size()].x;
        t_waypoints_y[i] = t_path[t_path.size()].y;
    }

    auto coeffs = polyfit(t_waypoints_x, t_waypoints_y, 3);
    double cte = polyeval(coeffs, 0);
    double epsi = -atan(coeffs[1]);

    // Predicting future states of the car to solve with optimizer.
    mpc.predict_FutureState(velocity_actual, steering_angle, cte, epsi, acc_Longi);

    Eigen::VectorXd state(6);

    // Pass to vehicles current sitiuation to state vector.
    state << pred_px, pred_py, pred_psi, velocity_actual, cte, epsi;

    // Pass the current state and reference trajector's coeffs to MPC Solver.
    auto output = mpc.Solve(state, coeffs);

    t_velocity += output[1] * dt;

    steer = output[0];

    sendCommandToSim(clip(-steer, -1, 1), t_velocity);
}

// Function Definitions for FG_eval Class.

void FG_eval::operator()(ADvector &fg, const ADvector &vars)
{
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.

    fg[0] = 0;

    // Reference State Cost
    // Below defines the cost related the reference state and
    // any anything you think may be beneficial.

    // Weights
    const int cte_cost_weight = 750;
    const int epsi_cost_weight = 750;
    const int v_cost_weight = 100;
    const int delta_cost_weight = 250;
    const int a_cost_weight = 100;
    const int delta_change_cost_weight = 250;
    const int a_change_cost_weight = 0;

    // Cost for CTE, psi error and velocity
    for (int t = 0; t < N; t++)
    {
        fg[0] += cte_cost_weight * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += v_cost_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Costs for steering (delta) and acceleration (a)
    for (int t = 0; t < N - 1; t++)
    {
        fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += a_cost_weight * CppAD::pow(vars[a_start + t], 2);
    }

    // Costs related to the change in steering and acceleration (makes the ride smoother)
    for (int t = 0; t < N - 2; t++)
    {
        fg[0] += delta_change_cost_weight * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += a_change_cost_weight * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Setup Model Constraints

    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++)
    {
        // State at time t + 1
        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> v1 = vars[v_start + t];
        AD<double> cte1 = vars[cte_start + t];
        AD<double> epsi1 = vars[epsi_start + t];

        // State at time t
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];
        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> epsi0 = vars[epsi_start + t - 1];

        // Actuator constraints at time t only
        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> a0 = vars[a_start + t - 1];

        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
        AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

        // Setting up the rest of the model constraints
        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
        fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
        fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
    }
}

// MPC Class Function Definitions

MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];    // Change in speed
    double cte = state[4];  // Deviation Error reference path
    double epsi = state[5]; // Orientetion Errror

    // Setting the number of model variables (includes both states and inputs).
    // N * state vector size + (N - 1) * 2 actuators (For steering & acceleration)
    size_t n_vars = N * state.size() + (N - 1) * 2;
    // Setting the number of constraints
    size_t n_constraints = N * state.size();

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0.0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Sets lower and upper limits for variables.
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (int i = delta_start; i < a_start; i++)
    {
        vars_lowerbound[i] = steer_lowerBound;
        vars_upperbound[i] = steer_upperBound;
    }

    // Acceleration/decceleration upper and lower limits.
    for (int i = a_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = acc_lowerBound;
        vars_upperbound[i] = acc_upperBound;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // Start lower and upper limits at current values
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;

    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";

    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.

    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // Returns the first actuator values, along with predicted x and y values to plot in the simulator
    vector<double> solved;
    solved.push_back(solution.x[delta_start]);
    solved.push_back(solution.x[a_start]);
    for (int i = 0; i < N; ++i)
    {
        solved.push_back(solution.x[x_start + i]);
        solved.push_back(solution.x[y_start + i]);
    }
    return solved;
}

void MPC::predict_FutureState(float velocity_actual, float delta, float cte, float epsi, float acc_Longi)
{
    pred_px = 0.0 + velocity_actual * dt;
    pred_py = 0.0;
    pred_psi = 0.0 + velocity_actual * -delta * dt / Lf;
    pred_v = velocity_actual + acc_Longi * dt;
    pred_cte = cte + velocity_actual * sin(epsi) * dt;
    pred_epsi = epsi + (velocity_actual * -delta * dt / Lf);
}
