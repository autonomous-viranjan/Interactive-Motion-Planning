#include <iostream>
#include <vector>

class Imputer
{
public:
    std::vector< std::vector<double> > impute(std::vector< std::vector<double> > &nv_trajectory, std::vector< std::vector<double> > &ego_trajectory);

private:
    const int r = 3; // time steps in the past observed
    const double alpha_scale = 100;
    const double L = 5.0/2.0; // vehicle length/2
    const double W = 0.5/2.0; // vehicle width wrt lane width/2
};