#include "gurobi_c++.h"
#include <vector>

using namespace std;

int main() // needs revision (chromatic number)
{
	int n = 4;
	
	vector<vector<int>> neighbours = {
		{1, 2,},
		{0,},
		{0, 3,},
		{2,},
	};
	
	GRBEnv env = GRBEnv(true);
	env.set("LogFile", "logs/chromatic_n.log");
	env.start();
	
	GRBModel model = GRBModel(env);
	
	vector<vector<GRBVar>> vars_x(n, vector<GRBVar>(n));
	vector<GRBVar> vars_y(n);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		vars_x[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + to_string(i) + "_" + to_string(j));
	}
	
	for (int j = 0; j < n; j++)
	vars_y[j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y_" + to_string(j));
	
	for (int i = 0; i < n; i++)
	{
		GRBLinExpr expr = 0;
		for (int j = 0; j < n; j++)
		expr += vars_x[i][j];
		model.addConstr(expr, GRB_EQUAL, 1, "C1_" + to_string(i));
	}
	
	for (int vertex = 0; vertex < n; vertex++)
	{
		for (auto neighbour : neighbours[vertex])
		{
			for (int j = 0; j < n; j++)
			model.addConstr(vars_x[vertex][j] + vars_x[neighbour][j] <= 1, "C2_" + to_string(vertex) + "_" + to_string(neighbour) + "_" + to_string(j));
		}
	}
	
	for (int vertex = 0; vertex < n; vertex++)
	for (int j = 0; j < n; j++)
	model.addConstr(vars_x[vertex][j] <= vars_y[j], "C2_" + to_string(vertex) + "_" + to_string(j));
	
	GRBLinExpr expr = 0;
	for (int j = 0; j < n; j++)
	expr += vars_y[j];
	model.setObjective(expr, GRB_MINIMIZE);
	
	model.optimize();
	
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		if (static_cast<int>(vars_x[i][j].get(GRB_DoubleAttr_X)) == 1)
		cout << vars_x[i][j].get(GRB_StringAttr_VarName) << endl;
	}
	
	for (int j = 0; j < n; j++) {
		cout << vars_y[j].get(GRB_StringAttr_VarName) << " "
		<< vars_y[j].get(GRB_DoubleAttr_X) << endl;
	}
	
	cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
	
	return 0;
}
