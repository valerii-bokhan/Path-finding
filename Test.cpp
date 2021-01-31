#include <iostream>
#include <chrono>

#include "BDAStar.h"

using namespace std;

int main()
{
	BDAStar finder(Heuristic::Octile, 1);

	{
		unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
		int pOutBuffer[12];
		int r = finder.FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);
		// 1, 5, 9

		cout << r << endl;

		for (int i = 0; i < r; i++)
			cout << pOutBuffer[i] << " ";
		cout << endl;
	}

	{
		unsigned char pMap[] = { 0, 0, 1, 0, 1, 1, 1, 0, 1 };
		int pOutBuffer[7];
		int r = finder.FindPath(2, 0, 0, 2, pMap, 3, 3, pOutBuffer, 7);
		// -1

		cout << r << endl;

		for (int i = 0; i < r; i++)
			cout << pOutBuffer[i] << " ";
		cout << endl;
	}

	// Memory Pool allocator test
	{
		const int count = 1000000;

		auto t1 = chrono::high_resolution_clock::now();
		
		for (int i = 0; i < count; i++)
			new Node(i-1, i+1, 1000);

		auto t2 = chrono::high_resolution_clock::now();

		auto duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1);

		cout << "Elapsed time: " << duration.count() << " ms\n";
	}

	return 0;
}
