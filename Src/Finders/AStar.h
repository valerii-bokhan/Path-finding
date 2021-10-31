#pragma once

#include "IFinder.h"
#include "Node.h"

#include <vector>
#include <queue>
#include <unordered_map>
#include <utility>

class AStar final : public IFinder
{
public:
    // Сlockwise order of direction 
    // to search for neighbors
    enum class ESide {
        Up,
        Right,
        Down,
        Left
    };

public:
    using HeuristicFunc = int(*)(int, int);

    explicit AStar(HeuristicFunc eHeuristic);
    AStar(HeuristicFunc eHeuristic, int nWeight);

public:
    // IFinder interface
    int FindPath(const int nStartX, const int nStartY,
        const int nTargetX, const int nTargetY,
        const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
        int* pOutBuffer, const int nOutBufferSize) override;

private:
    using NodeRef = std::reference_wrapper<Node>;

    int GetNeighbor(const Node& rCurrent, const ESide& eSide, std::pair<int, int>& point) const;

    Node& GetNode(int nIndex, int nX, int nY);
    Node& GetNode(int nX, int nY);
    Node& GetNode(int nIndex);

    const std::vector<int> BacktracePath(const Node& rTarget);

    int FillBuffer(const std::vector<int>& vPath,
        int* pOutBuffer, const int nOutBufferSize);

    int Heuristic(const Node& rNode, const int nTargetX, const int nTargetY);

    void Clear();

private:
    HeuristicFunc heuristic;

    int weight = 1;

    int mapWidth = 0;
    int mapHeight = 0;

    std::unordered_map<int, Node> nodes;
    std::priority_queue<NodeRef, std::vector<NodeRef>, Node::Compare> opened;
};
