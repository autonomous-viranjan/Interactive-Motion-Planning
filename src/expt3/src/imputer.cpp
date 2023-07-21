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

std::vector< std::vector<double> > Imputer::impute(std::vector< std::vector<double> > &nv_trajectory, std::vector< std::vector<double> > &ego_trajectory) 
{
    /*
        Input: Trajectory data of NV
            Trajectory given by [Xnv(k-r), ..., Xnv(k)] => rows->state & columns->time step
            where Xnv(k) = [s(k) v(k) a(k)]'

        Output: Weights [[alpha_v(k-r), alpha_a(k-r)], ..., [alpha_v(k), alpha_a(k)]]
    */
   std::vector< std::vector<double> > alphas;
   try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set("OutputFlag", "0");

        GRBVar Xnv[nx_nv][r];
        GRBVar alpha[2][r];
        GRBVar lambda[1][r];
        GRBVar nu[3][r];

        for (int t=0; t < r; t++) {
            Xnv[0][t] = model.addVar(-5.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            alpha[0][t] = model.addVar(0.0, alpha_scale, 0.0, GRB_CONTINUOUS);
            alpha[1][t] = model.addVar(0.0, alpha_scale, 0.0, GRB_CONTINUOUS);
            lambda[0][t] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);          
        }

        for (int t=0; t < r; t++) {
            model.addConstr(alpha[0][t] + alpha[1][t] == alpha_scale);
        }

        GRBQuadExpr Jnv = 0;
        for (int t=0; t < r; t++) {
            Jnv += (2/(L * L)) * (nv_trajectory[0][t] - ego_trajectory[0][t]) * (2/(L * L)) * (nv_trajectory[0][t] - ego_trajectory[0][t]) * lambda[0][t] * lambda[0][t] + nu[0][t] * nu[0][t] - (4/(L * L))*(nv_trajectory[0][t] - ego_trajectory[0][t]) * lambda[0][t] * nu[0][t]
            + 4 * (nv_trajectory[1][t] - vref) * (nv_trajectory[1][t] - vref) + nu[1][t] * nu[1][t] + 4 * (nv_trajectory[1][t] - vref) * alpha[0][t] * nu[1][t]
            + 4 * (nv_trajectory[2][t] * nv_trajectory[2][t]) * alpha[1][t] * alpha[1][t] + nu[2][t] * nu[2][t] + 4 * nv_trajectory[2][t] * nu[2][t]
            + (dt / tau) * (dt / tau) * nu[2][t] * nu[2][t]
            + lambda[0][t] * lambda[0][t] * (1 - ((nv_trajectory[0][t] - ego_trajectory[0][t]) / (L * L)) * ((nv_trajectory[0][t] - ego_trajectory[0][t]) / (L * L)) - ((2 - ego_trajectory[3][t]) / (W * W)) * ((2 - ego_trajectory[3][t]) / (W * W))) * (1 - ((nv_trajectory[0][t] - ego_trajectory[0][t]) / (L * L)) * ((nv_trajectory[0][t] - ego_trajectory[0][t]) / (L * L)) - ((2 - ego_trajectory[3][t]) / (W * W)) * ((2 - ego_trajectory[3][t]) / (W * W)));
        }

        model.setObjective(Jnv, GRB_MINIMIZE);
        model.set("TimeLimit", "0.2");
        model.optimize();

        int status = model.get(GRB_IntAttr_Status);

        if (status == GRB_OPTIMAL) {
            std::vector<double> alphas_v;
            std::vector<double> alphas_a;
            for (int t=0; t<r; t++) {
                alphas_v.push_back(alpha[0][t].get(GRB_DoubleAttr_X));
                alphas_a.push_back(alpha[1][t].get(GRB_DoubleAttr_X));
            }
            alphas.push_back(alphas_v);
            alphas.push_back(alphas_a);
        }
        else {
            // equal weight on both terms
            for (int t=0; t<r; t++) {
                alphas[0].push_back(alpha_scale / 2.0);
                alphas[1].push_back(alpha_scale / 2.0);
            }            
        }

   }
   catch(GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
   }

   return alphas;      
}