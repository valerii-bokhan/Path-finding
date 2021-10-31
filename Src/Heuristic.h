#pragma once

#define OCTILE_F 0.41421356237309504880

#include <algorithm>
#include <cmath>

// Basic heuristic functions

// Common params:
//	int dx - Difference in x.
//	int dy - Difference in y.

namespace Heuristic
{
	inline int Manhattan(int dx, int dy) { return dx + dy; }
	inline int Chebyshev(int dx, int dy) { return dx < dy ? dy : dx; }
	inline int Euclidean(int dx, int dy) { return static_cast<int>(std::sqrt(dx * dx + dy * dy)); }
	inline int Octile(int dx, int dy) { return static_cast<int>((dx < dy) ? OCTILE_F * dx + dy : OCTILE_F * dy + dx); }
	inline int Experimental(int dx, int dy) { return 7 * std::min(dx, dy) + 10 * (std::max(dx, dy) - std::min(dx, dy)); }
	inline int None(int dx, int dy) { return 0; }
}
