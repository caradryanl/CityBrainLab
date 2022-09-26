#ifndef CITY_BRAIN_COMPACTED_GRAPH_H_
#define CITY_BRAIN_COMPACTED_GRAPH_H_

#include <algorithm>
#include <map>
#include <vector>

class CompactedEdge {
public:
  int from_, to_;
  double time_;
  std::vector<int> path_;
  CompactedEdge() {}
};

class CompactedGraph {
public:
  std::vector<std::vector<std::pair<int, double>>> adj_;
  std::vector<std::vector<std::pair<int, double>>> rev_adj_;
  std::vector<std::vector<CompactedEdge>> compact_adj_;
  std::vector<int> lab_;
  std::map<std::pair<int, int>, CompactedEdge> compact_edge_table_;
  
  const std::vector<int> FindShortestPath(int from, int to, bool is_spfa = true) const;
  const std::vector<int> FindShortestPathBruteForce(int from, int to,
                                              bool is_spfa = true);
  double QueryPathLength(std::vector<int> path);
private:
  void InitFromAdjTable(std::vector<std::vector<std::pair<int, double>>> adj);
  void InitFromEdgeList(std::vector<std::pair<std::pair<int, int>, double>> edges, int n);

friend class RoadNet;
};

#endif // CITY_BRAIN_COMPACTED_GRAPH_H_
