#include "gurobi_c++.h"
#include <vector>

using namespace std;

int main()
{
	int n = 4;
	vector<vector<double>> cost_matrix = {
		{1.8, 2.8, 2.4, 2.5},
		{3.5, 2.4, 2.5, 1.0},
		{2.2, 1.3, 2.3, 1.1},
		{1.4, 1.0, 1.5, 1.4},
	};

	GRBEnv env = GRBEnv(true);
	env.set("LogFile", "logs/assignment.log");
	env.start();

	GRBModel model = GRBModel(env);

	vector<vector<GRBVar>> vars(n, vector<GRBVar>(n));
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < n; j++)
			vars[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + to_string(i) + "_" + to_string(j));
	}

	for (int i = 0; i < n; i++)
	{
		GRBLinExpr expr = 0;
		for (int j = 0; j < n; j++)
			expr += vars[i][j];
		model.addConstr(expr, GRB_EQUAL, 1.0, "P_" + to_string(i));
	}

	for (int j = 0; j < n; j++)
	{
		GRBLinExpr expr = 0;
		for (int i = 0; i < n; i++)
			expr += vars[i][j];
		model.addConstr(expr, GRB_EQUAL, 1.0, "J_" + to_string(j));
	}

	GRBLinExpr expr = 0;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
			expr += cost_matrix[i][j] * vars[i][j];
	}
	model.setObjective(expr, GRB_MINIMIZE);

	model.optimize();

	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cout << vars[i][j].get(GRB_StringAttr_VarName) << " "
			<< vars[i][j].get(GRB_DoubleAttr_X) << endl;
		}
	}
	
	cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

	return 0;
}