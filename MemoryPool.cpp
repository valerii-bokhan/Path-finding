#include "MemoryPool.h"

#include <mutex>

namespace
{
	std::mutex allocatorMutex;
}

PoolAllocator::PoolAllocator(size_t capacity)
	: blockCapacity(capacity)
	, head(nullptr)
{}

void* PoolAllocator::alloc(size_t size) noexcept
{
	if (head == nullptr)
		head = allocBlock(size);

	Chunk* firstFreeChunk = head;

	// Moving the head to the next free chunk
	head = head->next;

	return firstFreeChunk;
}

void PoolAllocator::free(void* pChunk, size_t size) noexcept
{
	std::lock_guard<std::mutex> guard(allocatorMutex);

	Chunk* chunk = reinterpret_cast<Chunk*>(pChunk);

	chunk->next = head;
	head = chunk;
}

Chunk* PoolAllocator::allocBlock(size_t capacity) noexcept
{
	std::lock_guard<std::mutex> guard(allocatorMutex);

	const size_t blockSize = blockCapacity * capacity;

	// Getting the memory
	Chunk* front = reinterpret_cast<Chunk*>(blocks.emplace_back(blockSize).data());

	// Building the singly linked list of memory chunks

	Chunk* chunk = front;
	for (int i = 0; i < blockCapacity - 1; ++i)
	{
		const auto shift = reinterpret_cast<std::byte*>(chunk) + capacity;
		chunk->next = reinterpret_cast<Chunk*>(shift);
		chunk = chunk->next;
	}

	// Closing the list
	chunk->next = nullptr;

	return front;
}
