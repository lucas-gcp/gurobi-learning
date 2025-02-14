#include "gurobi_c++.h"
#include <vector>
#include <cmath>
#include <cassert>

using namespace std;

double distance(double* x, double* y, int i, int j)
{
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

int main()
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
	env.set("LogFile", "logs/tsp.log");
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
