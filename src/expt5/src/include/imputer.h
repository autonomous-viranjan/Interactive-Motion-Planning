#pragma once

#include <iostream>
#include <vector>

const int r = 6; // time steps in the past observed

class Imputer
{
public:

    std::vector<double> impute(std::vector< std::vector<double> > &nv_trajectory, std::vector< std::vector<double> > &ego_trajectory);

private:
    
    const double alpha_scale = 100;
    const double L = 5.0/2.0; // vehicle length/2
    const double W = 0.5/2.0; // vehicle width wrt lane width/2
};