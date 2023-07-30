/*
    Imputer: imputes cost weights of NV model used in joint MPC by fitting an optimization problem to the observed trajectory of NV.
    Method based on the paper: 
    V. Bhattacharyya, A. Vahidi, "Automated Vehicle Highway Merging: Motion Planning via Adaptive Interactive Mixed-Integer MPC"
    
    Viranjan Bhattacharyya
    EMC2 Lab Clemson University
*/
#include "/home/optimal-student/vb/gurobi10.0.1_linux64/gurobi1001/linux64/include/gurobi_c++.h"
#include "include/imputer.h"
#include "include/jointmpc.h"

std::vector<double> Imputer::impute(std::vector< std::vector<double> > &nv_trajectory, std::vector< std::vector<double> > &ego_trajectory) 
{
    /*
        Input: Trajectory data of NV and Ego
            Trajectory given by 2D vector [X(k-r), ..., X(k)] => rows->state & columns->time step
            where X(k) = [s(k) v(k) a(k)] 1D vector

        Output: Weights [[alpha_v(k-r), alpha_a(k-r)], ..., [alpha_v(k), alpha_a(k)]]
    */
   std::vector<double> alphas;
   try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set("OutputFlag", "0");

        GRBVar Xnv[nx_nv][r];
        GRBVar alpha[2][1];
        GRBVar lambda[1][r];
        GRBVar nu[3][r];

        for (int t=0; t < r; t++) {
            Xnv[0][t] = model.addVar(-5.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            lambda[0][t] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            nu[0][t] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            nu[1][t] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            nu[2][t] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);          
        }
        alpha[0][0] = model.addVar(0.0, alpha_scale, 0.0, GRB_CONTINUOUS);
        alpha[1][0] = model.addVar(0.0, alpha_scale, 0.0, GRB_CONTINUOUS);

        model.addConstr(alpha[0][0] + alpha[1][0] == alpha_scale);

        GRBQuadExpr Jnv = 0;
        for (int t=0; t < r; t++) {
            Jnv += (2/(L * L)) * (nv_trajectory[t][0] - ego_trajectory[t][0]) * (2/(L * L)) * (nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] * lambda[0][t] + nu[0][t] * nu[0][t] - (4/(L * L))*(nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] * nu[0][t]
            + 4 * (nv_trajectory[t][1] - vref_) * (nv_trajectory[t][1] - vref_) * alpha[0][0] * alpha[0][0] + nu[1][t] * nu[1][t] + 4 * (nv_trajectory[t][1] - vref_) * alpha[0][0] * nu[1][t]
            + 4 * (nv_trajectory[t][2] * nv_trajectory[t][2]) * alpha[1][0] * alpha[1][0] + nu[2][t] * nu[2][t] + 4 * nv_trajectory[t][2] * alpha[1][0] * nu[2][t]
            + (dt / tau) * (dt / tau) * nu[2][t] * nu[2][t]
            + lambda[0][t] * lambda[0][t] * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W))) * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W)));
        }
        // // increased acceleration sensitivity
        // for (int t=0; t < r; t++) {
        //     Jnv += (2/(L * L)) * (nv_trajectory[t][0] - ego_trajectory[t][0]) * (2/(L * L)) * (nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] * lambda[0][t] + nu[0][t] * nu[0][t] - (4/(L * L))*(nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] * nu[0][t]
        //     + 4 * (nv_trajectory[t][1] - vref_) * (nv_trajectory[t][1] - vref_) * alpha[0][0] * alpha[0][0] + nu[1][t] * nu[1][t] + 4 * (nv_trajectory[t][1] - vref) * alpha[0][0] * nu[1][t]
        //     + 4 * (10 * nv_trajectory[t][2] * 10 * nv_trajectory[t][2]) * alpha[1][0] * alpha[1][0] + nu[2][t] * nu[2][t] + 4 * 10 * nv_trajectory[t][2] * alpha[1][0] * nu[2][t]
        //     + (dt / tau) * (dt / tau) * nu[2][t] * nu[2][t]
        //     + lambda[0][t] * lambda[0][t] * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W))) * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W)));
        // }
        // // decreased velocity sensitivity
        // for (int t=0; t < r; t++) {
        //     Jnv += (2/(L * L)) * (nv_trajectory[t][0] - ego_trajectory[t][0]) * (2/(L * L)) * (nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] * lambda[0][t] + nu[0][t] * nu[0][t] - (4/(L * L))*(nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] * nu[0][t]
        //     + 4 * (nv_trajectory[t][1] * 100 - vref) * (nv_trajectory[t][1] * 100 - vref) * alpha[0][0] * alpha[0][0] + nu[1][t] * nu[1][t] + 4 * (nv_trajectory[t][1] * 100 - vref) * alpha[0][0] * nu[1][t]
        //     + 4 * (nv_trajectory[t][2] * nv_trajectory[t][2]) * alpha[1][0] * alpha[1][0] + nu[2][t] * nu[2][t] + 4 * nv_trajectory[t][2] * alpha[1][0] * nu[2][t]
        //     + (dt / tau) * (dt / tau) * nu[2][t] * nu[2][t]
        //     + lambda[0][t] * lambda[0][t] * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W))) * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W)));
        // }
        // // decreased acceleration sensitivity
        // for (int t=0; t < r; t++) {
        //     Jnv += (2/(L * L)) * (nv_trajectory[t][0] - ego_trajectory[t][0]) * (2/(L * L)) * (nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] * lambda[0][t] + nu[0][t] * nu[0][t] - (4/(L * L))*(nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] * nu[0][t]
        //     + 4 * (nv_trajectory[t][1] - vref) * (nv_trajectory[t][1] - vref) * alpha[0][0] * alpha[0][0] + nu[1][t] * nu[1][t] + 4 * (nv_trajectory[t][1] - vref) * alpha[0][0] * nu[1][t]
        //     + 4 * ((nv_trajectory[t][2]/10) * (nv_trajectory[t][2]/10)) * alpha[1][0] * alpha[1][0] + nu[2][t] * nu[2][t] + 4 * (nv_trajectory[t][2]/10) * alpha[1][0] * nu[2][t]
        //     + (dt / tau) * (dt / tau) * nu[2][t] * nu[2][t]
        //     + lambda[0][t] * lambda[0][t] * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W))) * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W)));
        // }

        model.setObjective(Jnv, GRB_MINIMIZE);
        model.set("TimeLimit", "0.1");
        model.optimize();

        int status = model.get(GRB_IntAttr_Status);

        if (status == GRB_OPTIMAL) {
            alphas.push_back(alpha[0][0].get(GRB_DoubleAttr_X));
            alphas.push_back(alpha[1][0].get(GRB_DoubleAttr_X));
        }
        else {
            // equal weight on both terms            
            alphas.push_back(alpha_scale / 2.0);
            alphas.push_back(alpha_scale / 2.0);           
        }
   }
   catch(GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
   }

   return alphas;      
}