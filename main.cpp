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
#include "graphs.hh"
#include "wheeler_sort.hh"

bool is_wheeler(WDFA wdfa, vector<LL> SA){

    // Proof of correctness: Lemma 8 of the paper "Regular Languages Meet Prefix Sorting" in SODA.

    vector<LL> ISA = inverse_permutation(SA);

    vector<tuple<LL, LL, LL> > edges; // label, wheeler rank of origin, wheeler rank of destination

    for(LL v = 0; v < wdfa.n_nodes; v++){
        for(WDFA::Edge E : wdfa.succ[v]){
            edges.push_back({E.label, ISA[v], ISA[E.dest]});
        }
    }

    sort(edges.begin(), edges.end());
    bool good = true;
    for(LL i = 0; i < edges.size()-1; i++){
        LL a_i, u_i, v_i; tie(a_i, u_i, v_i) = edges[i];
        LL a_j, u_j, v_j; tie(a_j, u_j, v_j) = edges[i+1];
        if(a_i == a_j) good &= v_i <= v_j;
        if(a_i != a_j) good &= v_i < v_j;
    }
    return good;
}

void test_wheeler_sort(WDFA& wdfa){
    vector<LL> SA = wheeler_sort(wdfa);
    assert(is_wheeler(wdfa, SA));
}

int main(int argc, char** argv){

    if(argc != 1){
        cerr << "WheelerSort: Reads a deterministic Wheeler automaton from standard input and prints the Wheeler order of the nodes. The graph format is as follows: on the first line there are two space-separated integers: number of nodes n and number of edges m. The nodes of the automaton are then integers from 0 to n-1. On the next line there is one line which contains an integer that identifies the initial state of the automaton. Every state of the automaton must be reachable from the initial state. Then follows m lines, each defining one edge. An edge is defined with three space-separated values: source node, destination node, label. The edge label is a single ASCII character." << endl;
        return 1;
    }

    LL n, m, s; cin >> n >> m >> s;
    WDFA wdfa(n,s);
    for(LL i = 0; i < m; i++){
        LL u,v; cin >> u >> v;
        char c; cin >> c;
        wdfa.add_edge(u,v,c);
    }
    for(LL x : wheeler_sort(wdfa)) cout << x << " ";
    cout << endl;
    

}