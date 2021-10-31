#include "BDAStar.h"

#include <algorithm>
#include <mutex>

using namespace std;

namespace
{
    // Neighbor search directions (clockwise order)
    constexpr int directions[4][2] = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 } };

    mutex bufferMutex;
}

BDAStar::BDAStar(HeuristicFunc eHeuristic)
    : heuristic(eHeuristic)
{}

BDAStar::BDAStar(HeuristicFunc eHeuristic, int nWeight)
    : heuristic(eHeuristic)
    , weight(nWeight)
{}

int BDAStar::FindPath(const int nStartX, const int nStartY,
    const int nTargetX, const int nTargetY,
    const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
    int* pOutBuffer, const int nOutBufferSize)
{
    if (nStartX == nTargetX && nStartY == nTargetY)
        return 0;

    mapWidth = nMapWidth;
    mapHeight = nMapHeight;

    // Warm up the nodes container
    const int capacity = mapWidth * mapHeight;
    nodes.reserve(capacity);

    Node& rStart = GetNode(nStartX, nStartY);
    Node& rTarget = GetNode(nTargetX, nTargetY);

    // Prepare our starting node and open it
    openedForward.push(rStart);
    rStart.Open(true);

    // Prepare our target node and open it
    openedBackward.push(rTarget);
    rTarget.Open(false);

    // The main loop where we check all nodes that were marked to be visited
    while (!openedForward.empty() && !openedBackward.empty())
    {
        for (auto isForward : { true, false })
        {
            NodeQueue& queue = isForward ? openedForward : openedBackward;
            const Node* target = isForward ? &rTarget : &rStart;

            // Popping the front Node with the minimal F score
            Node& rCurrent = queue.top();
            queue.pop();

            rCurrent.Close();

            // Getting neighbors (clockwise order)
            for (auto direction : { ESide::Up, ESide::Right, ESide::Down, ESide::Left })
            {
                pair<int, int> point;
                const int index = GetNeighbor(rCurrent, direction, point);

                // Traversable locations of the grid are indicated by 1,
                // and impassable locations are indicated by 0.
                if (index >= 0 && pMap[index] == 1)
                {
                    Node& rNeighbor = GetNode(index, point.first, point.second);

                    if (rNeighbor.IsClosed())
                        continue;

                    // Checking if two finders are being met
                    if (rNeighbor.IsOpened() && rNeighbor.GetLead() == !isForward)
                    {
                        const vector<int> pathToStart = BacktracePath(isForward ? rCurrent : rNeighbor);
                        const vector<int> pathToTarget = BacktracePath(isForward ? rNeighbor : rCurrent);

                        const int result = FillBuffer(pathToStart, pathToTarget, pOutBuffer, nOutBufferSize);

                        // Clearing before next use
                        Clear();

                        // We have successfully found the path
                        return result;
                    }

                    // Diagonal movement disabled
                    // Nearest distance to the next neighbor = 1
                    const int ng = rCurrent.GetG() + 1;
                    const int nh = Heuristic(rNeighbor, target->GetX(), target->GetY());
                    const int nf = ng + nh;

                    // Check if we need to visit the neighbor and update its scores
                    if (!rNeighbor.IsOpened() || nf < rNeighbor.GetF())
                    {
                        rNeighbor.Update(ng, nh);
                        rNeighbor.SetParent(rCurrent.GetIndex());

                        queue.push(rNeighbor);
                        rNeighbor.Open(isForward);
                    }
                }
            }
        }
    }

    // Clearing before next use
    Clear();

    // We have failed to find the path
    return -1;
}

int BDAStar::GetNeighbor(const Node& rCurrent, const ESide& eSide, pair<int, int>& point) const
{
    const auto [i, j] = directions[static_cast<int>(eSide)];

    const int x = rCurrent.GetX() + i;
    const int y = rCurrent.GetY() + j;

    if (x >= 0 && y >= 0 && x < mapWidth && y < mapHeight)
    {
        point.first = x;
        point.second = y;

        return x + y * mapWidth;
    }

    return -1;
}

Node& BDAStar::GetNode(int nX, int nY)
{
    const int index = nX + nY * mapWidth;
    return GetNode(index, nX, nY);
}

Node& BDAStar::GetNode(int nIndex)
{
    const int x = nIndex % mapWidth;
    const int y = nIndex / mapWidth;
    return GetNode(nIndex, x, y);
}

Node& BDAStar::GetNode(int nIndex, int nX, int nY)
{
    auto [it, emplaced] = nodes.try_emplace(nIndex, nIndex, nX, nY);
    return it->second;
}

int BDAStar::Heuristic(const Node& rNode, const int nTargetX, const int nTargetY)
{
    const int dx = abs(rNode.GetX() - nTargetX);
    const int dy = abs(rNode.GetY() - nTargetY);

    return weight * heuristic(dx, dy);
}

void BDAStar::Clear()
{
    while (!openedForward.empty())
        openedForward.pop();

    while (!openedBackward.empty())
        openedBackward.pop();

    nodes.clear();
}

const vector<int> BDAStar::BacktracePath(const Node& rTarget)
{
    vector<int> path;

    const Node* node = &rTarget;
    path.push_back(node->GetIndex());
    int parent = node->GetParent();

    while (parent >= 0)
    {
        node = &GetNode(parent);
        path.push_back(node->GetIndex());
        parent = node->GetParent();
    }

    return path;
}

int BDAStar::FillBuffer(const vector<int>& vPathToStart, const vector<int>& vPathToTarget, int* pOutBuffer, const int nOutBufferSize)
{
    const int toStartSize = static_cast<int>(vPathToStart.size()) - 1;
    const int toTargetSize = static_cast<int>(vPathToTarget.size());

    const int pathSize = toStartSize + toTargetSize;

    if (pathSize > nOutBufferSize)
        return pathSize;

    lock_guard<mutex> guard(bufferMutex);

    for (int i = 0; i < toStartSize; i++)
    {
        const int index = toStartSize - i - 1;
        pOutBuffer[i] = vPathToStart[index];
    }

    for (int i = 0; i < toTargetSize; i++)
    {
        pOutBuffer[i + toStartSize] = vPathToTarget[i];
    }

    return pathSize;
}