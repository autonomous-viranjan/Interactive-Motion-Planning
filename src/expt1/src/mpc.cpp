/*
    Viranjan Bhattacharyya
    EMC2 Lab Clemson University
*/

#include "include/mpc.h"
#include <iostream>
#include <fstream>
#include <vector>
#include "/home/emc2/vb/gurobi10.0.3_linux64/gurobi1003/linux64/include/gurobi_c++.h"

std::vector<double> Mpc::sol(std::vector<double> &X0, 
                            std::vector<double> &s1_1_front, std::vector<double> &s1_1_rear,
                            std::vector<double> &s1_2_front, std::vector<double> &s1_2_rear,
                            double &s_obs)
{
    /*
        MPC input: current state (initial state), NV position, obstacle position
        MPC output: planned next state
    */
    std::vector<double> plan;

    try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set("OutputFlag", "0");    

        GRBVar X[nx][T];
        GRBVar U[nu][T];
        GRBVar mu[lanes][T];
        GRBVar beta[obs_count][T];
        GRBVar eps[2][T];

        // Define the variables
        for (int k = 0; k < T; k++) {
            X[0][k] = model.addVar(-5.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "s_" + std::to_string(k));
            X[1][k] = model.addVar(0.0, vmax, 0.0, GRB_CONTINUOUS, "v_" + std::to_string(k));
            X[2][k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "a_" + std::to_string(k));
            X[3][k] = model.addVar(lmin, lmax, 0.0, GRB_CONTINUOUS, "l_" + std::to_string(k));
            X[4][k] = model.addVar(-5.0, 5.0, 0.0, GRB_CONTINUOUS, "rl_" + std::to_string(k));
            U[0][k] = model.addVar(ua_min, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "ua_" + std::to_string(k));
            U[1][k] = model.addVar(1, 2, 0.0, GRB_INTEGER, "ul_" + std::to_string(k));
            mu[0][k] = model.addVar(0, 1, 0.0, GRB_BINARY, "mu1_" + std::to_string(k)); // lane indicator for lane 1
            mu[1][k] = model.addVar(0, 1, 0.0, GRB_BINARY, "mu2_" + std::to_string(k)); // lane indicator for lane 2         
            beta[0][k] = model.addVar(0, 1, 0.0, GRB_BINARY, "beta_" + std::to_string(k)); // front-back indicator for NV
            beta[1][k] = model.addVar(0, 1, 0.0, GRB_BINARY, "beta_" + std::to_string(k)); // front-back indicator for stopped vehicle
            eps[0][k] = model.addVar(0, 5, 0.0, GRB_CONTINUOUS); // slack var for NV constraint
            eps[1][k] = model.addVar(0, 2, 0.0, GRB_CONTINUOUS); // slack var for stopped veh constraint
        }

        // Define the dynamics
        for (int k = 0; k < T - 1; k++) {
            model.addConstr(X[0][k + 1] == X[0][k] + X[1][k] * dt);
            model.addConstr(X[1][k + 1] == X[1][k] + X[2][k] * dt);
            model.addConstr(X[2][k + 1] == X[2][k] + (-X[2][k]/tau + U[0][k]/tau) * dt);
            model.addConstr(X[3][k + 1] == X[3][k] + X[4][k] * dt);
            model.addConstr(X[4][k + 1] == X[4][k] + (-w_n * w_n * X[3][k] - 2 * zeta * w_n * X[4][k] + K * w_n * w_n * U[1][k]) * dt);
        }
        
        // initial state [s0 v0 a0 l0 rl0]'
        model.addConstr(X[0][0] == X0[0]);
        model.addConstr(X[1][0] == X0[1]);
        model.addConstr(X[2][0] == X0[2]);
        model.addConstr(X[3][0] == X0[3]);
        model.addConstr(X[4][0] == X0[4]);

        // Define constraints
        for (int k = 0; k < T; k++) {
            // Engine map      
            model.addConstr(U[0][k] <= m1 * X[1][k] + b1);
            model.addConstr(U[0][k] <= m2 * X[1][k] + b2);
            model.addConstr(X[2][k] <= m1 * X[1][k] + b1);
            model.addConstr(X[2][k] <= m2 * X[1][k] + b2);
            // Lane discipline
            model.addConstr(X[3][k] + bigM * mu[0][k] >= 2 - delta);
            model.addConstr(X[3][k] + bigM * mu[0][k] <= 2 - delta + bigM);
            model.addConstr(X[3][k] - bigM * mu[1][k] >= 1 + delta - bigM);
            model.addConstr(X[3][k] - bigM * mu[1][k] <= 1 + delta);            
            // Collision avoidance
            /* in lane 1 */
            model.addConstr(X[0][k] - s1_1_front[k] + eps[0][k] - bigM * beta[0][k] - bigM * mu[0][k] >= -2 * bigM);
            model.addConstr(s1_1_rear[k] - X[0][k] + eps[0][k] + bigM * beta[0][k] - bigM * mu[0][k] >= -bigM);
            //// obstacle
            model.addConstr(X[0][k] - (s_obs + 5) + eps[1][k] - bigM * beta[1][k] - bigM * mu[0][k] >= -2 * bigM);
            model.addConstr((s_obs - 10) - X[0][k] + eps[1][k] + bigM * beta[1][k] - bigM * mu[0][k] >= -bigM);
            /* in lane 2 */
            model.addConstr(X[0][k] - s1_2_front[k] + eps[0][k] - bigM * beta[0][k] - bigM * mu[1][k] >= -2 * bigM);
            model.addConstr(s1_2_rear[k] - X[0][k] + eps[0][k] + bigM * beta[0][k] - bigM * mu[1][k] >= -bigM);
        }

        // Define the objective
        GRBQuadExpr obj = 0;
        for (int k = 0; k < T; k++) {    
            obj += qv * (X[1][k] * X[1][k] - 2 * X[1][k] * vref) + qa * (X[2][k] * X[2][k]) + qa * (U[0][k] * U[0][k]) + 1e5 * eps[0][k] * eps[0][k] + 1e5 * eps[1][k] * eps[1][k];
        }
        for (int k = 1; k < T; k++) {
            obj += qul * (U[1][k] - U[1][k-1]) * (U[1][k] - U[1][k-1]) + qul * (X[3][k] - X[3][k-1]) * (X[3][k] - X[3][k-1]) + qda * (X[2][k] - X[2][k-1]) * (X[2][k] - X[2][k-1]);
        }
        model.setObjective(obj, GRB_MINIMIZE);
        // some settings
        model.set("TimeLimit", "0.2");
        // Optimize the model
        model.optimize();

        int status = model.get(GRB_IntAttr_Status);
        this->checkStatus(status);
        // std::cout << "Opt status:" << status << std::endl;

        if (status == GRB_OPTIMAL) {
            // Get solution
            plan.push_back(X[0][1].get(GRB_DoubleAttr_X));      
            plan.push_back(X[1][1].get(GRB_DoubleAttr_X));      
            plan.push_back(X[2][1].get(GRB_DoubleAttr_X));
            plan.push_back(X[3][1].get(GRB_DoubleAttr_X));
            plan.push_back(X[4][1].get(GRB_DoubleAttr_X));
            plan.push_back(U[0][0].get(GRB_DoubleAttr_X));
            plan.push_back(U[1][0].get(GRB_DoubleAttr_X));

            // for (int t=0; t < T; t++) {
            //     // std::cout << "beta: " << beta[1][t].get(GRB_DoubleAttr_X) << std::endl;
            //     // std::cout << "mu1: " << mu[0][t].get(GRB_DoubleAttr_X) << std::endl;
            //     // std::cout << "ul: " << U[1][t].get(GRB_DoubleAttr_X) << std::endl;
            //     // std::cout << "eps: " << eps[0][t].get(GRB_DoubleAttr_X) << std::endl;
            // }
        }
        else {
            // vehicle slow
            plan.push_back(X0[0]);      
            plan.push_back(2.5);      
            plan.push_back(1.0);
            plan.push_back(X0[3]);
            plan.push_back(0.0);

            plan.push_back(1.0);
            plan.push_back(X0[3]);
        }        

    } catch(GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch(...) {
        std::cout << "Exception during optimization" << std::endl;
    }          

    return plan;
}

void Mpc::checkStatus(const int & status) {
    if (status == GRB_OPTIMAL) this->write("Optimal.\n");
    else if (status == GRB_INFEASIBLE) this->write("Model is infeasible.\n");
    else if (status == GRB_UNBOUNDED) this->write("Model is unbounded.\n");
    else if (status == GRB_INF_OR_UNBD) this->write("Model is inf or unb.\n");
    else if (status == GRB_CUTOFF) this->write("Objective exceeded Cutoff.\n");
    else if (status == GRB_ITERATION_LIMIT) this->write("Barrier/Simplex iterations exceeded BarIterLimit.\n");
    else if (status == GRB_NODE_LIMIT) this->write("Nodes explored exceeded NodeLimit.\n");
    else if (status == GRB_TIME_LIMIT) this->write("Time exceeded TimeLimit.\n");
    else if (status == GRB_SOLUTION_LIMIT) this->write("Number of solutions reached SolutionLimit.\n");
    else if (status == GRB_INTERRUPTED) this->write("Optimization was terminated by user.\n");
    else if (status == GRB_SUBOPTIMAL) this->write("Unable to satisfy optimality tolerances.\n");
    else if (status == GRB_INPROGRESS) this->write("Solution in progress.\n");
    else if (status == GRB_USER_OBJ_LIMIT) this->write("Objective reached limit specified by user.\n");
}