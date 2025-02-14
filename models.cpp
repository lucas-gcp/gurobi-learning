#include "models.h"
#include "gurobi_c++.h"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cassert>

using namespace std;

int chromatic_n() // needs revision (chromatic number)
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

double distance(double* x, double* y, int i, int j) {
	double dx = x[i] - x[j];
	double dy = y[i] - y[j];

	return sqrt(dx*dx + dy*dy);
}

void find_subtour(int n, double** sol, vector<int> &tour, int &best_len)
{
	vector<int> current(n), seen(n, 0);
	int len, node, start = 0;
	best_len = n+1;

	while (start < n) {
		if (!seen[start]) {
			seen[start] = 1;
			len = 1;
			node = start;
			current[0] = start;
			for (int i = node + 1; i < n; i++) {
				if (sol[node][i] > 0.5 && !seen[i]) {
					current[len] = i;
					len++;
					seen[i] = 1;
					node = i;
					i = start;
				}
			}
			if (len < best_len) {
				tour = current;
				best_len = len;
			}
		}
		start++;
	}
}

class subtourelim: public GRBCallback
{
	public:
	GRBVar **vars;
	int n;
	subtourelim(GRBVar **xvars, int xn) {
		vars = xvars;
		n = xn;
	}
	protected:
	void callback() {
		try {
			if (where == GRB_CB_MIPSOL) {
				double **x = new double*[n];
				vector<int> tour(n);
				int len;

				for (int i = 0; i < n; i++) {
					x[i] = getSolution(vars[i], n);
				}

				find_subtour(n, x, tour, len);

				cout << len << endl;

				if (len < n) {
					GRBLinExpr expr = 0;
					for (int i = 0; i < len; i++)
						for (int j = i+1; j < len; j++)
							expr += vars[tour[i]][tour[j]];
					addLazy(expr <= len-1);
				}

				for (int i = 0; i < n; i++)
					delete[] x[i];
				delete x;
			}
		} catch (GRBException e) {
			cout << "Error number: " << e.getErrorCode() << endl;
			cout << e.getMessage() << endl;
		} catch (...) {
			cout << "Error during callback" << endl;	
		}
	}
};

int TSP() // Not done
{
	int n = 15;

	double* x = new double[n];
	double* y = new double[n];

	for (int i = 0; i < n; i++) {
		x[i] = ((double) rand())/RAND_MAX;
		y[i] = ((double) rand())/RAND_MAX;
	}

	for (int i = 0; i < n; i++)
		cout << x[i] << ", ";
	cout << endl;

	for (int i = 0; i < n; i++)
		cout << y[i] << ", ";
	cout << endl;

	GRBEnv env = GRBEnv(true);
	env.set("LogFile", "tsp.log");
	env.start();

	GRBModel model = GRBModel(env);

	model.set(GRB_IntParam_LazyConstraints, 1);

	vector<vector<GRBVar>> vars(n, vector<GRBVar>(n));
	for (int i = 0; i < n; i++) {
		for (int j = 0; j <= i; j++) {
			vars[i][j] = model.addVar(0.0, 1.0, distance(x, y, i, j), GRB_BINARY, "x_" + to_string(i) + "_" + to_string(j));
			vars[j][i] = vars[i][j];
		}
	}
	
	for (int i = 0; i < n; i++)
	{
		GRBLinExpr expr = 0;
		for (int j = 0; j < n; j++)
			expr += vars[i][j];
		model.addConstr(expr, GRB_EQUAL, 2, "deg2_" + to_string(i));
	}

	for (int i = 0; i < n; i++)
		vars[i][i].set(GRB_DoubleAttr_UB, 0);
		
	GRBVar** xvars = new GRBVar*[n];
	for (int i = 0; i < n; i++)
		xvars[i] = new GRBVar[n];
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			xvars[i][j] = vars[i][j];

	subtourelim cb = subtourelim(xvars, n);
	model.setCallback(&cb);
		
	model.optimize();

	if (model.get(GRB_IntAttr_SolCount) > 0) {
		double **sol = new double*[n];
		for (int i = 0; i < n; i++)
		  sol[i] = model.get(GRB_DoubleAttr_X, xvars[i], n);
  
		vector<int>tour(n);
		int len;
  
		find_subtour(n, sol, tour, len);
		assert(len == n);
  
		cout << "Tour: ";
		for (int i = 0; i < len; i++)
		  cout << tour[i] << " ";
		cout << endl;
  
		for (int i = 0; i < n; i++)
		  delete[] sol[i];
		delete[] sol;
	}

	delete[] xvars; 

	cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

	return 0;
}

int knapsack() // needs revision
{
	int n = 4;
	vector<double> weights = { 1.8, 2.8, 2.4, 2.5 };
	vector<double> values = { 3.5, 2.4, 2.5, 1.0 };
	double W = 5.8;

	GRBEnv env = GRBEnv(true);
	env.set("LogFile", "knapsack.log");
	env.start();

	GRBModel model = GRBModel(env);

	vector<GRBVar> vars(n);
	for (int i = 0; i < n; i++)
		vars[i] = model.addVar(0.0, n, 0.0, GRB_INTEGER, "x_" + to_string(i));

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

int knapsack_0_1()
{
	int n = 4;
	vector<double> weights = { 1.8, 2.8, 2.4, 2.5 };
	vector<double> values = { 3.5, 2.4, 2.5, 1.0 };
	double W = 5.8;

	GRBEnv env = GRBEnv(true);
	env.set("LogFile", "knapsack.log");
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

int assignment()
{
	int n = 4;
	vector<vector<double>> cost_matrix = {
		{1.8, 2.8, 2.4, 2.5},
		{3.5, 2.4, 2.5, 1.0},
		{2.2, 1.3, 2.3, 1.1},
		{1.4, 1.0, 1.5, 1.4},
	};

	GRBEnv env = GRBEnv(true);
	env.set("LogFile", "assignment.log");
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