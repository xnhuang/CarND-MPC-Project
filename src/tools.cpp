//
// Created by xnhuang on 6/20/17.
//

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// For converting back and forth between radians and degrees.
double Tools::pi() { return M_PI; }
double Tools::deg2rad(double x) { return x * this->pi() / 180; }
double Tools::rad2deg(double x) { return x * 180 / this->pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string Tools::hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double Tools::polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Tools::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

//convert reference into local coordinate
void Tools::coordinate_transform(const double x, const double y,const double theta,
                                 const vector<double> xvals, const vector<double> yvals,
                                 vector<double>& xvals_new, vector<double>& yvals_new) {
    auto n = xvals.size();
    for(int i=0;i<n;i++){
        xvals_new.push_back(cos(theta)*(xvals[i]-x)+sin(theta)*(yvals[i]-y));
        yvals_new.push_back(-sin(theta)*(xvals[i]-x)+cos(theta)*(yvals[i]-y));
    }
}