#pragma once

#include <vector>
#include <cassert>
#include <string>
#include <iostream>
#include <utility>
#include <map>
#include <deque>
#include <queue>
#include <set>
#include <fstream>
#include <functional>

using namespace std;

typedef long long LL;

class WDFA{ // Wheeler DFA

    public:

    struct Edge{
        LL dest;
        LL label;
        Edge(LL dest, LL label) : dest(dest), label(label) {}
    };

    LL n_nodes;
    LL n_edges;
    LL source_id;

    vector<vector<Edge> > succ; // node id -> list of node ids of successors
    vector<vector<Edge> > pred; // node id -> list of node ids of predecessors

    // Constructor for WDFA with n_nodes nodes and no edges
    WDFA(LL n_nodes, LL source_id) : n_nodes(n_nodes), n_edges(0), source_id(source_id), succ(n_nodes), pred(n_nodes){}

    void add_edge(LL from, LL to, LL label){
        succ[from].push_back(Edge(to, label));
        pred[to].push_back(Edge(from, label));
        n_edges++;
    }

};
