/* 
 * FILE --- rk4_sim.hh
 *
 * PURPOSE --- Header file for a class for a 4'th order Runge-Kutta 
 * method for solving a system of  n first order differential equations. 
 * If the initial equation is in the form of n'th order differential equation
 * it must be converted to a system of n first order differential
 * equations.
 *
 */

// Prevent Multiple Inclusion
#ifndef RK4_SIM_HH
#define RK4_SIM_HH

// Include Files
#include <stdlib.h>

// Local Definitions
#define RK4_SIM_NO_ERROR 0
#define RK4_SIM_MEM_ERROR 1
#define RK4_SIM_UNKNOWN_IC_ERROR 2


class RK4_SIM
{
public:
  // Constructor
  // The arguments are self-explanatory
  // If you specify initial conditions, the parameter _initial_state 
  // should point to a vector of size _plant_order, so that correct 
  // initialization could be performed.
  RK4_SIM(int _plant_order,
	  int _ctrl_order,
	  double _integration_step = 0.0001, 
	  double _initial_time = 0.0, 
	  double *_initial_state = NULL);  

  // Destructor
  virtual ~RK4_SIM();
  
  // Initilaizes simulation parameters
  void init_param(double _integration_step, 
		  double _initial_time, 
		  double *_initial_state);  

  void solve (void); // Performs one step simulation
  
  // DERIV contains the dynamic equations of the system in the 
  // form: xdot = f(t,x,u); arguments are the time t, present state 
  // values x, the present control value u, and values of the derivatives 
  // of the states xdot (calculated inside of the funtcion). 
  // It is overloaded in the derived class!!!
  virtual void DERIV(const double t, const double *x, 
		     const double *u, double *xdot) {};
  

  double *plant_state; // Current System states
  double *ctrl_input; // Current Controller output
  double current_time;  // Current time
  int rk4_sim_err;   // Variable to hold an error number
  
protected:
  int plant_order; // Num of plant states
  int ctrl_order; // Num of control inputs
  double integration_step; // Integration step size

private:
  // Allocate/free memory for the arrays
  void allocate_memory(void);
  void free_memory(void);

  double *f1, *f2, *f3, *f4, *temp; // Runge-Kutta Coefficients

 // Functions which calculate RK coefficients
  inline void F1 (void); 
  inline void F2 (void);
  inline void F3 (void);
  inline void F4 (void);
};

#endif // RK4_SIM_HH


