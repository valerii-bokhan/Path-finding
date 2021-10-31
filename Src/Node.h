#pragma once

#include "Utils/MemoryPool.h"
#include "Heuristic.h"

#define MP_BLOCK_CAPACITY 64

class Node final
{
public:
	struct Compare final
	{
		bool operator()(const Node& lhs, const Node& rhs) const
		{
			return lhs.GetF() > rhs.GetF();
		}
	};

	static PoolAllocator& GetAllocator()
	{
		static PoolAllocator allocator{ MP_BLOCK_CAPACITY };
		return allocator;
	}

	static void* operator new(size_t size) {
		return GetAllocator().alloc(size);
	}

	static void operator delete(void* ptr, size_t size) {
		return GetAllocator().free(ptr, size);
	}

public:
	Node(const int nIndex, const int nX, const int nY)
		: index(nIndex)
		, x(nX)
		, y(nY)
	{}

public:
	void Update(int nG, int nH) { g = nG; h = nH; };

	void SetParent(int pParent) { parentIndex = pParent; }
	int GetParent() const { return parentIndex; }

	void Close() { closed = true; }
	const bool IsClosed() const { return closed; }

	void Open(bool bLead) { opened = true; headForward = bLead; }
	const bool IsOpened() const { return opened; }
	const bool GetLead() const { return headForward; }

	const int GetIndex() const { return index; }
	const int GetF() const { return g + h; }
	const int GetG() const { return g; }

	const int GetX() const { return x; }
	const int GetY() const { return y; }

private:
	int x;
	int y;

	int g = 0;
	int h = 0;

	int index;
	int parentIndex = -1;

	bool closed = false;
	bool opened = false;
	bool headForward = true;
};
