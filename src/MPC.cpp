#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 10;
double dt = 0.2;
double ref_v = 200. * (4./9.);

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// variables vector mapping of index to variable
// this makes the code a lot easier to read and understand

// STATE variables = x, y, psi, v
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;

// ERROR variables = cross track error and orientation error
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;

// ACTUATORS = delta (throttle) and a (steering angle)
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    // ************************************************************************
    // *  COSTS                                                               *
    // ************************************************************************

    // The cost is stored in the first element of the cost and constraints
    // vector. 
    // All changes to the COST should happen on fg[0]
    // fg[1:N] will hold the CONSTRAINTS
    fg[0] = 0; 

    // Costs associated with the reference state
    for (int t = 0; t < N; t++) {
      // Cross track squared error
      fg[0] += 5000 * CppAD::pow(vars[cte_start + t], 2);
      // Orientation squared error
      fg[0] += 500 * CppAD::pow(vars[epsi_start + t], 2);
      // Velocity squared error - ensures car won't find a solution
      // where it doesn't move, but is the simplest solution to deal with
      // this issue.
      // We may want to explore other ways such as use the euclidean dist
      // between the vehicle and the destination
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Costs associated with constant change of throttle and steering
    // we want the drive to be smooth in both dimensions
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 50000 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Costs associated with rate of change in actuators values
    // this penalizes behaviors such as drastic changes in steering
    // angle or throttle
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 5000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // ************************************************************************
    // *  CONSTRAINTS                                                         *
    // ************************************************************************

    // Since our fg vector has the cost at position 0
    // we will add 1 to each of the other variables starting index
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Now the rest of the constraints
    for (int t = 1; t < N; t++) {
      // we store the variables for easy reading of the constraints

      // Variables at time t
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // Variables at time t-1
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Actuators contraints are limited at each observation
      // so we only need to store them at time t.
      // the other restrictions (for smoothness) were added
      // to the cost function
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // Cost constraint
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0,2) + coeffs[3] * pow(x0,3);

      // Desired steering angle
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2. * coeffs[2] * x0 + 3. * coeffs[3] * pow(x0,2));

      // STATE constraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

      // COSTS constraints
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Store variables for better readability
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // Store state variables for easy reading of formulas
  size_t n_vars = 6 * N + 2 * (N-1);

  // Set the number of constraints = state_variables * N
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.

  // CAN REPLACE THIS WITH FILL?
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;

  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;

  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;

  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;

  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;

  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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

  vector<double> output = {solution.x[delta_start], solution.x[a_start], N};

  for (i = 0; i < N; ++i) {
    output.push_back(solution.x[x_start + i]);
  }

  for (i = 0; i < N; ++i) {
    output.push_back(solution.x[y_start + i]);
  }

  return output;

}
