#include <vector>
#include <iostream>

using namespace std;

void
findsubtour(int      n,
            double** sol,
            int*     tourlenP,
            int*     tour)
{
  bool* seen = new bool[n];
  int bestind, bestlen;
  int i, node, len, start;

  for (i = 0; i < n; i++)
    seen[i] = false;

  start = 0;
  bestlen = n+1;
  bestind = -1;
  node = 0;
  while (start < n) {
    for (node = 0; node < n; node++)
      if (!seen[node])
        break;
    if (node == n)
      break;
    for (len = 0; len < n; len++) {
      tour[start+len] = node;
      seen[node] = true;
      for (i = 0; i < n; i++) {
        if (sol[node][i] > 0.5 && !seen[i]) {
          node = i;
          break;
        }
      }
      if (i == n) {
        len++;
        if (len < bestlen) {
          bestlen = len;
          bestind = start;
        }
        start += len;
        break;
      }
    }
  }

  for (i = 0; i < bestlen; i++)
    tour[i] = tour[bestind+i];
  *tourlenP = bestlen;

  delete[] seen;
}

int main()
{
    int n = 12;
	vector<vector<int>> sol = {
		{0, 0, -0, -0, -0, 0, -0, 1, 1, -0, -0, -0, },
		{0, 0, -0, 1, -0, 1, -0, -0, 0, -0, -0, -0, },
		{-0, -0, 0, -0, 1, -0, 1, -0, -0, -0, -0, -0, },
		{-0, 1, -0, 0, -0, 1, -0, -0, -0, -0, -0, -0, },
		{-0, -0, 1, -0, 0, -0, -0, -0, -0, -0, 1, -0, },
		{0, 1, -0, 1, -0, 0, -0, -0, 0, -0, -0, -0, },
		{-0, -0, 1, -0, -0, -0, 0, -0, -0, -0, 1, -0, },
		{1, -0, -0, -0, -0, -0, -0, 0, 0, -0, -0, 1, },
		{1, 0, -0, -0, -0, 0, -0, 0, 0, 1, -0, -0, },
		{-0, -0, -0, -0, -0, -0, -0, -0, 1, 0, -0, 1, },
		{-0, -0, -0, -0, 1, -0, 1, -0, -0, -0, 0, -0, },
		{-0, -0, -0, -0, -0, -0, -0, 1, -0, 1, -0, 0, },
	};
	// Gurobi example output for comparison
	for (int i = 0; i < n; i++) {
		cout << i << ": ";
		for (int j = 0; j < n; j++) {
			if (sol[i][j] == 1)
				cout << j << ", ";
		}
		cout << endl;
	}

	double **sol_d = (double **)malloc(n * sizeof(double*));
	for (int i = 0; i < n; i++)
		sol_d[i] = (double*)malloc(n * sizeof(double));
	
	for (int i = 0; i < n; i++)
		for (int j = 0; j < n; j++)
			sol_d[i][j] = sol[i][j];
	
	int tourlenP, *tour2 = new int[n];
	findsubtour(n, sol_d, &tourlenP, tour2);
	
	for (int i = 0; i < tourlenP; i++)
		cout << tour2[i] << " ";
	cout << endl;

	for (int i = 0; i < n; i++)
		free(sol_d[i]);
	free(sol_d);
	
	
		
	vector<int> tour, current(n), seen(n, 0);
	tour.reserve(n);
	int len, node, best_len = n+1, start = 0;
	cout << sol [10][6] << endl;
	while (start < n) {
		if (!seen[start]) {
			seen[start] = 1;
			len = 1;
			node = start;
			current[0] = start;
			for (int i = node + 1; i < n; i++) {
				if (sol[node][i] > 0.5 && !seen[i]) {
					cout << node << " " << i << endl;
					current[len] = i;
					len++;
					seen[i] = 1;
					node = i;
					i = start;
				}
			}
			cout << "start: " << start << endl;
			for (int i = 0; i < len; i++)
				cout << current[i] << ", ";
			cout << endl << "len:" << len << endl;
			if (len < best_len) {
				tour = current;
				best_len = len;
			}
		}
		start++;
	}

	for (int i = 0; i < best_len; i++)
		cout << tour[i] << " ";
	cout << endl;

    cout << best_len << endl;

    return 0;
}