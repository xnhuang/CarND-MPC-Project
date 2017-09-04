//
// Created by xnhuang on 6/20/17.
//

#ifndef MPC_TOOLS_H
#define MPC_TOOLS_H
#include <vector>
#include "Eigen/Dense"
using Eigen::VectorXd;
using Eigen::MatrixXd;

class Tools {
public:
    Tools();
    virtual ~Tools();
    // For converting back and forth between radians and degrees.
    double pi();
    double deg2rad(double x);
    double rad2deg(double x);

    // Checks if the SocketIO event has JSON data.
    // If there is data the JSON object in string format will be returned,
    // else the empty string "" will be returned.
    std::string hasData(std::string s);

    // Evaluate a polynomial.
    double polyeval(Eigen::VectorXd coeffs, double x);

    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                            int order);

    // estimate CTE
    void coordinate_transform(const double x, const double y,const double theta,
                              const std::vector<double> xvals, const std::vector<double> yvals,
                              std::vector<double>& xvals_new, std::vector<double>& yvals_new);
};
#endif //MPC_TOOLS_H
