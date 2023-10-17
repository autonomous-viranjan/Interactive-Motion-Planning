/*
    Viranjan Bhattacharyya
    EMC2 Lab Clemson University
*/

#include "include/jointmpc.h"
#include "/home/emc2desktop/vb/gurobi10.0.3_linux64/gurobi1003/linux64/include/gurobi_c++.h"

std::vector<double> Mpc::sol(std::vector<double> &X0, std::vector<double> &X0_NV, double &s_obs, double &alpha_v, double &alpha_a, std::ofstream &horizonFile)
{
    /*
        Joint MPC input: current state of Ego and NV
        MPC output: planned next state
    */
    std::vector<double> plan;

    try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set("OutputFlag", "0");    

        GRBVar X[nx][T];
        GRBVar Xnv[nx_nv][T];
        GRBVar U[nu][T];
        GRBVar Unv[nu_nv][T];
        GRBVar mu[lanes][T];
        GRBVar beta[obs_count][T];
        GRBVar eps[2][T];

        // Define the variables
        for (int k = 0; k < T; k++) {
            X[0][k] = model.addVar(-5.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            X[1][k] = model.addVar(0.0, vmax, 0.0, GRB_CONTINUOUS);
            X[2][k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            X[3][k] = model.addVar(lmin, lmax, 0.0, GRB_CONTINUOUS);
            X[4][k] = model.addVar(-5.0, 5.0, 0.0, GRB_CONTINUOUS);

            Xnv[0][k] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            Xnv[1][k] = model.addVar(0.0, vmax, 0.0, GRB_CONTINUOUS);
            Xnv[2][k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS);

            U[0][k] = model.addVar(ua_min, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
            U[1][k] = model.addVar(1, 2, 0.0, GRB_INTEGER);

            Unv[0][k] = model.addVar(ua_min, -ua_min, 0.0, GRB_CONTINUOUS);

            mu[0][k] = model.addVar(0, 1, 0.0, GRB_BINARY);
            mu[1][k] = model.addVar(0, 1, 0.0, GRB_BINARY);
            beta[0][k] = model.addVar(0, 1, 0.0, GRB_BINARY);
            beta[1][k] = model.addVar(0, 1, 0.0, GRB_BINARY);
            eps[0][k] = model.addVar(0, 5, 0.0, GRB_CONTINUOUS);
            eps[1][k] = model.addVar(0, 5, 0.0, GRB_CONTINUOUS);
        }

        // Define the dynamics
        for (int k = 0; k < T - 1; k++) {
            // Ego
            model.addConstr(X[0][k + 1] == X[0][k] + X[1][k] * dt);
            model.addConstr(X[1][k + 1] == X[1][k] + X[2][k] * dt);
            model.addConstr(X[2][k + 1] == X[2][k] + (-X[2][k] / tau + U[0][k] / tau) * dt);
            model.addConstr(X[3][k + 1] == X[3][k] + X[4][k] * dt);
            model.addConstr(X[4][k + 1] == X[4][k] + (-w_n * w_n * X[3][k] - 2 * zeta * w_n * X[4][k] + K * w_n * w_n * U[1][k]) * dt);
            // NV as assumed by Ego
            model.addConstr(Xnv[0][k + 1] == Xnv[0][k] + Xnv[1][k] * dt);
            model.addConstr(Xnv[1][k + 1] == Xnv[1][k] + Xnv[2][k] * dt);
            model.addConstr(Xnv[2][k + 1] == Xnv[2][k] + (-Xnv[2][k] / tau + Unv[0][k] / tau) * dt);            
        }
        
        // Ego initial state [s0 v0 a0 l0 rl0]'
        model.addConstr(X[0][0] == X0[0]);
        model.addConstr(X[1][0] == X0[1]);
        model.addConstr(X[2][0] == X0[2]);
        model.addConstr(X[3][0] == X0[3]);
        model.addConstr(X[4][0] == X0[4]);
        // NV (actual) initial state
        model.addConstr(Xnv[0][0] == X0_NV[0]);
        model.addConstr(Xnv[1][0] == X0_NV[1]);
        model.addConstr(Xnv[2][0] == X0_NV[2]);

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
            // in lane 1
            // model.addConstr(X[0][k] - s1_1_front[k] + eps[0][k] - bigM * beta[0][k] - bigM * mu[0][k] >= -2 * bigM);
            // model.addConstr(s1_1_rear[k] - X[0][k] + eps[0][k] + bigM * beta[0][k] - bigM * mu[0][k] >= -bigM);
            //// obstacle
            model.addConstr(X[0][k] - (s_obs + 5) + eps[1][k] - bigM * beta[1][k] - bigM * mu[0][k] >= -2 * bigM);
            model.addConstr((s_obs - 10) - X[0][k] + eps[1][k] + bigM * beta[1][k] - bigM * mu[0][k] >= -bigM);
            // in lane 2
            // model.addConstr(X[0][k] - s1_2_front[k] - bigM * beta[0][k] - bigM * mu[1][k] >= -2 * bigM);
            // model.addConstr(s1_2_rear[k] - X[0][k] + bigM * beta[0][k] - bigM * mu[1][k] >= -bigM);
            model.addConstr(X[0][k] - (Xnv[0][k] + 5) - bigM * beta[0][k] - bigM * mu[1][k] >= -2 * bigM);
            model.addConstr((Xnv[0][k] - 5) - X[0][k] + bigM * beta[0][k] - bigM * mu[1][k] >= -bigM);
        }

        // Define the objective
        GRBQuadExpr obj = 0;
        //// Same nature (weights) of Ego as NV
        // for (int k = 0; k < T; k++) {    
        //     obj += alpha_v * (X[1][k] * X[1][k] - 2 * X[1][k] * vref) + alpha_a * (X[2][k] * X[2][k]) + alpha_a * (U[0][k] * U[0][k]) + 1e5 * eps[0][k] * eps[0][k] + 1e5 * eps[1][k] * eps[1][k]
        //         + alpha_v * (Xnv[1][k] * Xnv[1][k] - 2 * Xnv[1][k] * vref) + alpha_a * (Xnv[2][k] * Xnv[2][k]) + alpha_a * (Unv[0][k] * Unv[0][k]);
        // }
        // Predefined nature (weights) of Ego
        for (int k = 0; k < T; k++) {    
            obj += qv * (X[1][k] * X[1][k] - 2 * X[1][k] * vref) + qa * (X[2][k] * X[2][k]) + qa * (U[0][k] * U[0][k]) + 1e5 * eps[0][k] * eps[0][k] + 1e5 * eps[1][k] * eps[1][k]
                + alpha_v * (Xnv[0][k] - X[0][k]) * (Xnv[0][k] - X[0][k]) + alpha_a * (Xnv[2][k] * Xnv[2][k]) + alpha_a * (Unv[0][k] * Unv[0][k]);
        }
        //// delta costs
        for (int k = 1; k < T; k++) {
            obj += qul * (U[1][k] - U[1][k-1]) * (U[1][k] - U[1][k-1]) + qul * (X[3][k] - X[3][k-1]) * (X[3][k] - X[3][k-1]) + qda * (X[2][k] - X[2][k-1]) * (X[2][k] - X[2][k-1])
                + alpha_a * (Xnv[2][k] - Xnv[2][k-1]) * (Xnv[2][k] - Xnv[2][k-1]);
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
            // next step prediction of NV
            plan.push_back(Xnv[0][1].get(GRB_DoubleAttr_X));
            plan.push_back(Xnv[1][1].get(GRB_DoubleAttr_X));
            plan.push_back(Xnv[2][1].get(GRB_DoubleAttr_X));

            // log horizon data            
            if (horizonFile.is_open()) {
                for (int i=0; i < T; i++) {
                    horizonFile <<  X[0][i].get(GRB_DoubleAttr_X)  << " " << X[1][i].get(GRB_DoubleAttr_X) << " " << X[2][i].get(GRB_DoubleAttr_X) << " " << X[3][i].get(GRB_DoubleAttr_X) << " " << X[4][i].get(GRB_DoubleAttr_X) << " " << Xnv[0][i].get(GRB_DoubleAttr_X) << " " << Xnv[1][i].get(GRB_DoubleAttr_X) << " " << Xnv[2][i].get(GRB_DoubleAttr_X) << std::endl;           
                }
            }
            else {std::cout << "Unable to open horizon file" << std::endl;}
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
            // no prediction
            plan.push_back(0.0);
            plan.push_back(0.0);
            plan.push_back(0.0);
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