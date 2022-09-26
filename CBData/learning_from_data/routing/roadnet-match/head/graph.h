//
// Created by Apia Du on 2020/11/11.
//

#ifndef CITY_BRAIN_COMPACTED_GRAPH_H_
#define CITY_BRAIN_COMPACTED_GRAPH_H_

#include <algorithm>
#include <map>
#include <vector>
#include <stack>
#include <cstring>

#include "utils.h"

class CompactedEdge {
public:
    int from_, to_;
    double time_;
    std::vector<int> path_;
    CompactedEdge() {}
};

class CompactedGraph {
private:
    std::vector<std::vector<std::pair<int, double>>> adj_;
    std::vector<std::vector<std::pair<int, double>>> rev_adj_;
    std::vector<std::vector<CompactedEdge>> compact_adj_;
    std::vector<int> lab_;
    std::map<std::pair<int, int>, CompactedEdge> compact_edge_table_;

    void InitFromAdjTable(std::vector<std::vector<std::pair<int, double>>> &adj);
    void InitFromEdgeList(std::vector<std::pair<std::pair<int, int>, double>> &edges, int n);
public:
    CompactedGraph(std::vector<std::pair<std::pair<int, int>, double>> &edges, int n);
    const std::vector<int> FindShortestPathBruteForce(int from, int to) const;
    const std::vector<int> FindShortestPath(int from, int to, bool is_spfa = true) const;
    double QueryPathLength(const std::vector<int> &path, int size = INT32_MAX);
    double QueryRoadLength(int from, int to);
    const std::vector<int> DFS(int from, int to) const;
};

#endif // CITY_BRAIN_COMPACTED_GRAPH_H_
