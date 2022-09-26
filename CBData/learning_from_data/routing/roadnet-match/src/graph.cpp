 //
// Created by Apia Du on 2020/11/11.
//

#include "../head/graph.h"
#include <functional>
#include <iostream>
#include <queue>
#include <set>

CompactedGraph::CompactedGraph(std::vector<std::pair<std::pair<int, int>, double>> &edges, int n)
{
    InitFromEdgeList(edges, n);
}

void CompactedGraph::InitFromAdjTable(std::vector<std::vector<std::pair<int, double>>> &adj)
{
    adj_ = adj;
    compact_edge_table_.clear();
    rev_adj_.clear();
    compact_adj_.clear();
    lab_.clear();
    // process
    int n = adj_.size(), m = 0;
    lab_ = std::vector<int>(n, -1);
    rev_adj_ = std::vector<std::vector<std::pair<int, double>>>(
                n, std::vector<std::pair<int, double>>(0));
    for (int i = 0; i < n; i++)
        if (adj_[i].size() > 2) {
            lab_[i] = m++;
        }
    for (int i = 0; i < n; i++) {
        for (auto edge : adj_[i]) {
            rev_adj_[edge.first].push_back({i, edge.second});
        }
    }
    compact_adj_ =
            std::vector<std::vector<CompactedEdge>>(m, std::vector<CompactedEdge>(0));
    for (int i = 0; i < n; i++)
        if (lab_[i] != -1) {
            std::vector<int> path;
            std::function<void(int, int, double)> dfs = [&](int u, int pre,
                    double times) {
                path.push_back(u);
                if (lab_[u] != -1 && pre != -1) {
                    CompactedEdge edge;
                    edge.from_ = lab_[i];
                    edge.to_ = lab_[u];
                    edge.time_ = times;
                    edge.path_ = path;
                    compact_adj_[lab_[i]].push_back(edge);
                    std::pair<int, int> edge_pair(lab_[i], lab_[u]);
                    if (!compact_edge_table_.count(edge_pair) ||
                            compact_edge_table_[edge_pair].time_ > edge.time_) {
                        compact_edge_table_[{lab_[i], lab_[u]}] = edge;
                    }
                } else {
                    for (auto edge : adj_[u]) {
                        int v = edge.first;
                        if (v == pre)
                            continue;
                        dfs(v, u, times + edge.second);
                    }
                }
                path.pop_back();
            };
            dfs(i, -1, 0);
        }
}

void CompactedGraph::InitFromEdgeList(std::vector<std::pair<std::pair<int, int>, double>> &edges, int n)
{
    std::vector<std::vector<std::pair<int, double>>> adj;
    adj.resize(n);
    for (auto edge : edges) {
        adj[edge.first.first].push_back({edge.first.second, edge.second});
        adj[edge.first.second].push_back({edge.first.first, edge.second});
    }
    InitFromAdjTable(adj);
}

const std::vector<int> CompactedGraph::FindShortestPath(int from, int to, bool is_spfa) const
{
    int m = compact_adj_.size();
    std::vector<double> dis(m, 1e30);
    std::vector<bool> vis(m, false);
    std::vector<int> pre(m, -2);
    std::set<int> tmp_edge;
    std::vector<int> path;
    std::map<std::pair<int, int>, CompactedEdge> extra_edge_table_;
    double ans_dis = 1e30;
    std::vector<int> ans_path;
    std::function<void(int, int, double)> dfs = [&](int u, int pr, double times) {
        path.push_back(u);
        if (u == to && times < ans_dis) {
            ans_dis = times;
            ans_path = path;
        }
        if (lab_[u] != -1) {
            if (dis[lab_[u]] > times) {
                CompactedEdge edge;
                edge.from_ = -1;
                edge.to_ = lab_[u];
                edge.time_ = times;
                edge.path_ = path;
                extra_edge_table_[{-1, lab_[u]}] = edge;
                dis[lab_[u]] = times;
                pre[lab_[u]] = -1;
                tmp_edge.insert(u);
            }
        } else {
            for (auto edge : adj_[u]) {
                int v = edge.first;
                if (v == pr)
                    continue;
                dfs(v, u, times + edge.second);
            }
        }
        path.pop_back();
    };
    dfs(from, -1, 0);
    if (is_spfa == false) {
        std::set<std::pair<double, int>> pq;
        for (int i = 0; i < m; i++)
            pq.insert({dis[i], i});
        for (int i = 0; i < m; i++) {
            int u = pq.begin()->second;
            pq.erase(pq.begin());
            vis[u] = true;
            for (const CompactedEdge &road : compact_adj_[u]) {
                int v = road.to_;
                if (dis[v] > dis[u] + road.time_) {
                    pq.erase({dis[v], v});
                    pre[v] = u;
                    dis[v] = dis[u] + road.time_;
                    pq.insert({dis[v], v});
                }
            }
        }
    } else {
        std::queue<int> q;
        for (int i = 0; i < m; i++)
            if (dis[i] < 1e20) {
                q.push(i);
                vis[i] = true;
            }
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            for (const CompactedEdge &road : compact_adj_[u]) {
                int v = road.to_;
                if (dis[v] > dis[u] + road.time_) {
                    pre[v] = u;
                    dis[v] = dis[u] + road.time_;
                    if (!vis[v]) {
                        vis[v] = true;
                        q.push(v);
                    }
                }
            }
            vis[u] = false;
        }
    }
    auto expand_path = [&](int v) {
        std::vector<int> path;
        while (true) {
            const std::map<std::pair<int, int>, CompactedEdge> *query_table =
                    (pre[v] == -1 ? &extra_edge_table_ : &compact_edge_table_);
            if (!query_table->count({pre[v], v})) {
                throw std::invalid_argument("Can't find the edge {pre[v],v} in  "
                                            "CompactedGraph::FindShortestPath()");
            }
            auto edge = (*query_table).at({pre[v], v}).path_;
            std::reverse(edge.begin(), edge.end());
            for (auto node : edge)
                path.push_back(node);
            v = pre[v];
            if (v != -1) {
                path.pop_back();
            } else {
                break;
            }
        }
        reverse(path.begin(), path.end());
        return path;
    };
    std::function<void(int, int, double)> dfs_rev = [&](int u, int pr,
            double times) {
        path.push_back(u);
        if (lab_[u] != -1) {
            if (times + dis[lab_[u]] < ans_dis) {
                auto tmp_path = expand_path(lab_[u]);
                auto tmp_path2 = path;
                tmp_path2.pop_back();
                reverse(tmp_path2.begin(), tmp_path2.end());
                for (auto node : tmp_path2)
                    tmp_path.push_back(node);
                ans_dis = times + dis[lab_[u]];
                ans_path = tmp_path;
            }
        } else {
            for (auto edge : rev_adj_[u]) {
                int v = edge.first;
                if (v == pr)
                    continue;
                dfs_rev(v, u, times + edge.second);
            }
        }
        path.pop_back();
    };
    path.clear();
    dfs_rev(to, -1, 0);
//    if (ans_dis > 1e20) {
//        throw std::invalid_argument(
//                    "Unknown error in  CompactedGraph::FindShortestPath()");
//    }
    return ans_path;
}

const std::vector<int> CompactedGraph::FindShortestPathBruteForce(int from, int to) const
{
    int m = adj_.size();
    std::vector<double> dis(m, 1e30);
    std::vector<bool> vis(m, false);
    std::vector<int> pre(m, -1);
    dis[from] = 0;
    std::set<std::pair<double, int>> pq;
    for (int i = 0; i < m; i++)
        pq.insert({dis[i], i});
    for (int i = 0; i < m; i++) {
        int u = pq.begin()->second;
        pq.erase(pq.begin());
        vis[u] = true;
        for (auto road : adj_[u]) {
            int v = road.first;
            if (dis[v] > dis[u] + road.second) {
                pq.erase({dis[v], v});
                pre[v] = u;
                dis[v] = dis[u] + road.second;
                pq.insert({dis[v], v});
            }
        }
    }
    std::vector<int> path;
    int v = to;
    while (true) {
        path.push_back(v);
        if (v == from)
            break;
        v = pre[v];
    }
    reverse(path.begin(), path.end());
    return path;
};

double CompactedGraph::QueryPathLength(const std::vector<int> &path, int size) {
    size = (size < (int)path.size())? size : (int)path.size();
    double length = 0;
    for (int i = 0; i + 1 < size; i++) {
        int u = path[i], v = path[i + 1];
        if (u >= (int)adj_.size() || v >= (int)adj_.size()) {
            throw std::invalid_argument(
                        "Label id is greater than the number of vertices in  "
                        "CompactedGraph::QueryPathLength()");
        }
        bool found = false;
        for (auto edge : adj_[u]) {
            if (edge.first == v) {
                found = true;
                length += edge.second;
                break;
            }
        }
        if (!found) {
            throw std::invalid_argument(
                        "Edge is not found in CompactedGraph::QueryPathLength()");
        }
    }
    return length;
}

double CompactedGraph::QueryRoadLength(int from, int to)
{
    if (from >= (int)adj_.size() || to >= (int)adj_.size()) {
        throw std::invalid_argument(
                    "Label id is greater than the number of vertices in  "
                    "CompactedGraph::QueryPathLength()");
    }
    for (auto edge : adj_[from]) {
        if (edge.first == to) {
            return edge.second;
        }
    }
    throw std::invalid_argument(
                "Edge is not found in CompactedGraph::QueryPathLength()");
}

const std::vector<int> CompactedGraph::DFS(int from, int to) const
{
    int node_num = adj_.size();
    bool visited[node_num];
    std::stack<int> visit_stack;
    std::vector<int> path;
    memset(visited, false, sizeof(bool) * node_num);

    visit_stack.push(from);

    while (!visit_stack.empty())
    {
        int visiting=visit_stack.top();
        visit_stack.pop();
        if (visited[visiting])
        {
            continue;
        }
        else
        {
            std::cout << visiting << " ";
            visited[visiting]=true;
            std::vector<std::pair<int, double>>::const_iterator node_it = adj_[visiting].begin();
            while (node_it < adj_[visiting].end())
            {
                visit_stack.push(node_it->first);
                if (to == node_it->first)
                {
                    return path;
                }
                ++node_it;
            }
        }
    }
    std::cout << std::endl;
    return path;
}
