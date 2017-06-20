
 #ifndef MPC_H
 #define MPC_H
 
 #include <vector>
 #include "Eigen-3.3/Eigen/Core"
 
 using namespace std;
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
const double latency = 0.1;


 class MPC {
 public:
   double last_delta;
   double last_a  ;


 MPC();
 
 virtual ~MPC();
 
 // Solve the model given an initial state and polynomial coefficients.
 // Return the first actuatotions.
 vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
 
   
 
 };

 #endif /* MPC_H */
