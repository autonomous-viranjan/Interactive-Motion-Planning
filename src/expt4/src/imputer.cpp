/*
    Imputer: estimates neighbor behavior by estimating cost weights
        OCP fit to trajectory data to determine nature of NV:
            J_hat = alpha_risk (s_NV - s_ego)^2 + alpha_safe (a_NV)^2
    
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
            Trajectory given by 2D vector [X(k-r), ..., X(k)] => access: trajectory[time_step][state]
            where X(k) = [s(k) v(k) a(k)] 1D vector

        Output: Weights alpha[0] = alpha_risk, alpha[1] = alpha_safe
    */
   std::vector<double> alphas;
   try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set("OutputFlag", "0");

        GRBVar alpha[2][1];
        GRBVar lambda[1][r];
        GRBVar nu[3][r];

        for (int t=0; t < r; t++) {
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
            Jnv += (2 * alpha[0][0] * (nv_trajectory[t][0] - ego_trajectory[t][0]) - 2 * (nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] / (L * L) + nu[0][t]) * (2 * alpha[0][0] * (nv_trajectory[t][0] - ego_trajectory[t][0]) - 2 * (nv_trajectory[t][0] - ego_trajectory[t][0]) * lambda[0][t] / (L * L) + nu[0][t])
            + (nu[1][t]) * (nu[1][t])
            + (2 * alpha[1][0] * nv_trajectory[t][2] + nu[2][t]) * (2 * alpha[1][0] * nv_trajectory[t][2] + nu[2][t])
            + (-dt * nu[2][t] / tau) * (-dt * nu[2][t] / tau)
            + lambda[0][t] * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W))) * lambda[0][t] * (1 - ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) * ((nv_trajectory[t][0] - ego_trajectory[t][0]) / (L * L)) - ((2 - ego_trajectory[t][3]) / (W * W)) * ((2 - ego_trajectory[t][3]) / (W * W)));
        }

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