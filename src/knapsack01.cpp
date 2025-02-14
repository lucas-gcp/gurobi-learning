#include "gurobi_c++.h"
#include <vector>

using namespace std;

int main()
{
	int n = 4;
	vector<double> weights = { 1.8, 2.8, 2.4, 2.5 };
	vector<double> values = { 3.5, 2.4, 2.5, 1.0 };
	double W = 5.8;

	GRBEnv env = GRBEnv(true);
	env.set("LogFile", "logs/knapsack01.log");
	env.start();

	GRBModel model = GRBModel(env);

	vector<GRBVar> vars(n);
	for(int i = 0; i < n; i++)
		vars[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_" + to_string(i));

	GRBLinExpr expr = 0;
	for (int i = 0; i < n; i++)
		expr += weights[i] * vars[i];

	model.addConstr(expr, GRB_LESS_EQUAL, W, "C");

	expr = 0;
	for (int i = 0; i < n; i++)
		expr += values[i] * vars[i];

	model.setObjective(expr, GRB_MAXIMIZE);

	model.optimize();

	for (int i = 0; i < n; i++)
	{
		cout << vars[i].get(GRB_StringAttr_VarName) << " "
			 << vars[i].get(GRB_DoubleAttr_X) << endl;
	}

	cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

	return 0;
}
