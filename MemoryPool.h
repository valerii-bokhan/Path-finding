#pragma once

#include <stdlib.h>
#include <vector>

// Singly linked list of memory chunks
struct Chunk
{
	Chunk* next = nullptr;
};

class PoolAllocator final {
public:
	explicit PoolAllocator(size_t capacity);

	// Returns the first free chunk
	// Allocates a new block of memory if needed
	void* alloc(size_t size) noexcept;

	// Binds free chunk to the beginning of the list
	void free(void* pPtr, size_t size) noexcept;

private:
	// Allocates a block of memory as a singly linked list of chunks
	// Returns a pointer to the first chunk in the list
	Chunk* allocBlock(size_t capacity) noexcept;

	std::vector<std::vector<std::byte>> blocks;

	size_t blockCapacity;

	Chunk* head;
};
