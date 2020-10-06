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
#include <tuple>
#include "graphs.hh"

using namespace std;

typedef long long LL;

class BFS{

private:

  BFS(const BFS&) = delete; // No copying because have a pointer member
  void operator=(const BFS&) = delete; // No copying because have a pointer member

public:
    queue<pair<LL,LL> > Q; // pairs node, depth
    vector<bool> visited;
    WDFA* wdfa;
    LL v, d; // Current element, depth
    BFS(WDFA* wdfa) : wdfa(wdfa) {
        Q.push({wdfa->source_id,0});
        visited.resize(wdfa->n_nodes);
    }

    // Returns true if next element was found, otherwise false.
    // The element will be put to this->v and the depth to this->d
    bool next(){
        v = -1; d = -1;
        if(Q.size() == 0) return false;

        // Find the first non-visited node
        while(Q.size() > 0){
            tie(v,d) = Q.front(); Q.pop();
            if(!visited[v]){
                visited[v] = true;
                for(WDFA::Edge e : wdfa->succ[v]) Q.push({e.dest, d + 1});
                return true;
            }
        }
        return false; // All nodes in Q were already visited
    }
};

// Stable sort a vector v of elements with integer keys from [0, max_key]
// T is the type of the elements in v
// func is function type that takes a type T and returns the integer key
template<typename T, typename func>
void stable_sort(vector<T>& v, func get_key, LL max_key){
    vector<LL> key_counts(max_key+1);
    for(LL i = 0; i < v.size(); i++) key_counts[get_key(v[i])]++;
    vector<LL> C = {0}; // C[i] = number of keys strictly smaller than i
    for(LL key = 1; key <= max_key; key++) C.push_back(C.back() + key_counts[key-1]);
    
    vector<T> v2(v.size());
    for(LL i = 0; i < v.size(); i++) v2[C[get_key(v[i])]++] = v[i];
    v = v2;
}

vector<LL> inverse_permutation(vector<LL>& p){
    vector<LL> pinv(p.size());
    for(LL i = 0; i < p.size(); i++)
        pinv[p[i]] = i;
    return pinv;
}

// Returns the graph built of nodes that have depth not equal to k mod 3
pair<WDFA, vector<LL>> get_G_new(WDFA& wdfa, LL k){

    auto inlabel = [&](LL v){
        if(wdfa.pred[v].size() == 0) return -1ll; // Something smaller than all labels
        return wdfa.pred[v][0].label;
    };

    // If can't go back, returns v
    auto go_back = [&](LL v){
        if(wdfa.pred[v].size() == 0) return v; // Something smaller than all labels
        return wdfa.pred[v][0].dest;
    };

    vector<LL> node_mapping(wdfa.n_nodes,-1); // node_mapping[v] = id of node v in the new graph.
    typedef pair<tuple<LL,LL,LL>, pair<LL,LL> > E; 
    vector<E> new_edges; // backward path label of length 3, (u,v)

    BFS bfs(&wdfa);
    LL n_nodes_in_new_graph = 0;
    LL max_c = 0;
    LL min_c = 1e18;
    while(bfs.next()){
        LL v = bfs.v; LL d = bfs.d;
        if(d % 3 != k || d == 0){
            node_mapping[v] = n_nodes_in_new_graph++; // Add to new graph
            LL c1 = inlabel(v);
            LL c2 = inlabel(go_back(v));
            LL c3 = inlabel(go_back(go_back(v)));
            LL u = go_back(go_back(go_back(v)));
            if(d != 0) new_edges.push_back({{c1,c2,c3}, {node_mapping[u], node_mapping[v]}});
            max_c = max(max_c, max(c1, max(c2,c3)));
            min_c = min(min_c, min(c1, min(c2,c3)));
        }
    }

    // Radix sort
    stable_sort(new_edges, [&](const E& p){return get<2>(p.first) - min_c;}, max_c-min_c);
    stable_sort(new_edges, [&](const E& p){return get<1>(p.first) - min_c;}, max_c-min_c);
    stable_sort(new_edges, [&](const E& p){return get<0>(p.first) - min_c;}, max_c-min_c);

    LL lex_name = 0;
    WDFA G_new(n_nodes_in_new_graph, 0);
    for(LL i = 0; i < new_edges.size(); i++){
        if(i > 0 && new_edges[i].first != new_edges[i-1].first) lex_name++;
        LL u = new_edges[i].second.first;
        LL v = new_edges[i].second.second;
        G_new.add_edge(u,v,lex_name);
    }
    return {G_new, node_mapping};
}

vector<LL> get_depths(WDFA& wdfa){
    vector<LL> depths(wdfa.n_nodes);
    BFS bfs(&wdfa);
    while(bfs.next()) depths[bfs.v] = bfs.d;
    return depths;
}

vector<LL> get_depth_k_nodes_in_order(WDFA& wdfa, vector<LL>& ISA_new, vector<LL>& to_G_new, LL k){
    
    BFS bfs(&wdfa);
    vector<tuple<LL,LL,LL> > names; // (label(v), label(pred(v)), v
    LL min_c = 1e18, max_c = 0;
    LL min_u = 1e18, max_u = 0;
    while(bfs.next()){
        LL v = bfs.v; LL d = bfs.d;
        if(d % 3 == k){
            LL c = -1; LL u = -1; // These will be left as-is for the root
            if(d != 0){
                c = wdfa.pred[v][0].label;
                u = wdfa.pred[v][0].dest;
                u = ISA_new[to_G_new[u]];
            }
            names.push_back({c, u, v});
            max_c = max(max_c, c); min_c = min(min_c, c);
            max_u = max(max_u, u); min_u = min(min_u, u);
        }
    }

    
    // Sort names with two linear stable sorts (radix sort). 
    // Map all keys from [min,max] to [0,max-min]
    stable_sort(names, [&min_u](const tuple<LL,LL,LL>& p){return get<1>(p) - min_u;}, max_u-min_u);
    stable_sort(names, [&min_c](const tuple<LL,LL,LL>& p){return get<0>(p) - min_c;}, max_c-min_c);

    vector<LL> ans;
    for(auto X : names) ans.push_back(get<2>(X));
    
    return ans;
}

vector<LL> brute_force_sort(WDFA& wdfa, LL maxdepth){
    vector<pair<vector<LL>,LL> > back_paths; // path, node
    for(LL v = 0; v < wdfa.n_nodes; v++){
        vector<LL> path;
        LL u = v;
        LL depth = 0;
        while(depth <= maxdepth && wdfa.pred[u].size() > 0){
            path.push_back(wdfa.pred[u][0].label);
            u = wdfa.pred[u][0].dest;
            depth++;
        }
        back_paths.push_back({path,v});
    }
    sort(back_paths.begin(), back_paths.end());
    vector<LL> W(wdfa.n_nodes); // Wheeler order permutation
    for(LL i = 0; i < wdfa.n_nodes; i++){
        W[i] = back_paths[i].second;
    }
    return W;
}

vector<LL> wheeler_sort(WDFA wdfa){

    vector<LL> depths = get_depths(wdfa);
    LL brute_threshold = 3;
    if(*max_element(depths.begin(), depths.end()) <= brute_threshold) // Recursion base case.
        return brute_force_sort(wdfa, brute_threshold);

    vector<LL> n_nodes_at_depth(3);
    for(LL v = 0; v < wdfa.n_nodes; v++)
        n_nodes_at_depth[depths[v] % 3]++;
    
    // Let k be argmax(n_nodes_at_depth):
    LL k = max_element(n_nodes_at_depth.begin(), n_nodes_at_depth.end()) - n_nodes_at_depth.begin();

    // The recursive call takes all nodes that are at depth not equal to k mod 3.

    WDFA G_new(0,0); vector<LL> to_G_new;
    std::tie(G_new, to_G_new) = get_G_new(wdfa,k);

    vector<LL> to_G(G_new.n_nodes);
    for(LL v = 0; v < wdfa.n_nodes; v++){
        if(depths[v] % 3 != k) to_G[to_G_new[v]] = v;
    }

    vector<LL> SA_new = wheeler_sort(G_new); // "Suffix array of G_new"
    assert(SA_new.size() == G_new.n_nodes);
    vector<LL> ISA_new = inverse_permutation(SA_new);
    vector<LL> order_notk;
    for(LL i = 0; i < SA_new.size(); i++){
        if(i == 0 && k == 0) continue; // Don't take in the added artificial extra root
        order_notk.push_back(to_G[SA_new[i]]);
    }
    
    vector<LL> order_k = get_depth_k_nodes_in_order(wdfa, ISA_new, to_G_new, k);

    assert(order_k.size() + order_notk.size() == wdfa.n_nodes);

    // Define function to for comparing nodes at depth k mod 3 with nodes at depth not k mod 3
    // First, two helpers:
    auto inlabel = [&](LL v){
        if(wdfa.pred[v].size() == 0) return -1ll; // Something smaller than all labels
        return wdfa.pred[v][0].label;
    };

    // If can't go back, returns v
    auto go_back = [&](LL v){
        if(wdfa.pred[v].size() == 0) return v; // Something smaller than all labels
        return wdfa.pred[v][0].dest;
    };

    // A comparison function that goes back one step
    auto comp_prev = [&](LL u, LL v){
        LL u_c1 = inlabel(u);
        LL u_prev = go_back(u);

        LL v_c1 = inlabel(v);
        LL v_prev = go_back(v);

        if(u_c1 != v_c1) return u_c1 < v_c1;
        return ISA_new[to_G_new[u_prev]] < ISA_new[to_G_new[v_prev]];
    };

    // A comparison function that goes back two steps
    auto comp_prev2 = [&](LL u, LL v){ 
        LL u_c1 = inlabel(u);
        LL u_c2 = inlabel(go_back(u));
        LL u_prev = go_back(go_back(u));

        LL v_c1 = inlabel(v);
        LL v_c2 = inlabel(go_back(v));
        LL v_prev = go_back(go_back(v));

        if(u_c1 != v_c1) return u_c1 < v_c1;
        if(u_c2 != v_c2) return u_c2 < v_c2;
        return ISA_new[to_G_new[u_prev]] < ISA_new[to_G_new[v_prev]];
    };

    // Merge the orders for depth k mod 3 and depth not k mod 3
    vector<LL> ans;
    LL i_k = 0;
    LL i_notk = 0;
    while(i_k < order_k.size() || i_notk < order_notk.size()){
        if(i_k == order_k.size()) ans.push_back(order_notk[i_notk++]);
        else if(i_notk == order_notk.size()) ans.push_back(order_k[i_k++]);
        else{
            LL v_k = order_k[i_k];
            LL v_notk = order_notk[i_notk];
            if(depths[v_notk] % 3 == (k+1) % 3){
                if(comp_prev2(v_k, v_notk)) ans.push_back(order_k[i_k++]);
                else ans.push_back(order_notk[i_notk++]);
            } else{
                if(comp_prev(v_k, v_notk)) ans.push_back(order_k[i_k++]);
                else ans.push_back(order_notk[i_notk++]);     
            }
        }
    }

    return ans;
}