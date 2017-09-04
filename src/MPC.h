#ifndef MPC_H
#define MPC_H
#include "tools.h"

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
    size_t N;
    double dt;
    double Lf;
    double ref_v;
    double a_initial;
    double delta_initial;
    // reference fit degree
    int polyfit_degree;
    public:
        MPC(size_t N, double dt, double Lf, double ref_v, int polyfit_degree, double a_init,double delta_init);
        int get_polyfit_degree();
        double get_a_init();
        double get_delta_init();
        double get_Lf();
        void set_a_init(double a_init);
        void set_delta_init(double delta_init);
        virtual ~MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs,
        vector<double> &x_mpc_log, vector<double> &y_mpc_log);
};

#endif /* MPC_H */
