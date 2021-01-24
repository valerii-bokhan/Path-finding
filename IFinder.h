﻿#pragma once

class IFinder
{
public:
	virtual ~IFinder() = default;

	virtual int FindPath(const int nStartX, const int nStartY,
		const int nTargetX, const int nTargetY,
		const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
		int* pOutBuffer, const int nOutBufferSize) = 0;
};
