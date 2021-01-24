#include <iostream>

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

	return 0;
}
