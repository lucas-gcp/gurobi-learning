#include <vector>
#include <iostream>

using namespace std;

int main()
{
    int n = 11;
	vector<vector<int>> sol = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
		{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1},
		{0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
		{0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0},
		{0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1},
		{0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0},
		{0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0},
	};

	vector<int> tour, current(n), seen(n, 0);
	tour.reserve(n);
	int len, node, best_len = n+1, start = 0;

	cout << best_len << endl;
	while (start < n) {
		cout << start << endl;
		if (!seen[start]) {
			seen[start] = 1;
			len = 0;
			node = start;
			current[0] = start;
			for (int i = node + 1; i < n; i++) {
				if (sol[node][i] && !seen[i]) {
					len++;
					current[len] = i;
					seen[i] = 1;
					node = i;
				}
				for (auto vertex : current)
					cout << vertex << " ";
				cout << endl;
			}
			cout << len << endl;
			if (len < best_len) {
				tour = current;
				best_len = len;
			}
		}
		start++;
	}

    for (auto i : tour)
        cout << i << " ";
	cout << endl;
    cout << best_len << endl;

    return 0;
}