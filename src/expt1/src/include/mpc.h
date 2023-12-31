#pragma once

#include <vector>
#include <iostream>

// GLOBAL VARS
const int T = 20; // horizon length
const double dt = 0.2; // sampling time
const double vref = 10.0; // reference speed

class Mpc
{
public:    
    std::vector<double> sol(std::vector<double> &initial_state, 
                            std::vector<double> &s1_1_front, std::vector<double> &s1_1_rear,
                            std::vector<double> &s1_2_front, std::vector<double> &s1_2_rear,
                            double &s_obs);

    void write(const char* s) {
        std::cout << s << std::endl;
    }
    void checkStatus(const int &status);

private:

    // Simulation parameters
    int nx = 5; // number of states
    int nu = 2; // number of inputs

    int lref = 1;
    int lanes = 2;
    int obs_count = 2;

    double qv = 10.0;    
    double qa = 30.0;
    double qul = 1000.0;
    double qda = 100.0; 

    int bigM = 1e5;    
    double delta = 0.5;
    double vmax = 36; // m/s
    double ua_min = -5;  // m/s^2;
    double lmin = 0.5;
    double lmax = 2.5;
    double m1 = 0.285;
    double b1 = 2.0;
    double m2 = -0.1208;
    double b2 = 4.83;

    // Model params
    // double tau = 0.275;
    double tau = 0.1;
    double zeta = 1;
    double w_n = 1.091;
    double K = 1.0; 
};