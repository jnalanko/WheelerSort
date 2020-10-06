# Wheeler sorting a deterministic Wheeler automaton

Linear-time sorting of deterministic Wheeler automata. Beta version. 

## Compiling

Requires a compiler with support for c++17. There are no external dependencies. To compile, just run `make`.

## Usage.

Reads a deterministic Wheeler automaton from standard input and prints the Wheeler order of the nodes. The graph format is as follows: on the first line there are two space-separated integers: number of nodes n and number of edges m. The nodes of the automaton will be integers from 0 to n-1. On the next line there is one line which contains an integer that identifies the initial state of the automaton. Every state of the automaton must be reachable from the initial state. Then follows m lines, each defining one edge. An edge is defined with three space-separated values: source node, destination node, label. The edge label is a single ASCII character.

Prints to standard out put n space-separated integers, where the i-th integer is the rank of node i in the Wheeler order.

# Example

Below is an example input:

```
6 5
0
0 1 a
0 2 b
1 3 b
2 4 a
2 5 c
```

This defines an automaton 6 nodes and 5 edges. The initial state is 0. The edges are (0,1,a), (0,2,b), (1,3,b), (2,4,a), (2,5,c). This input is in the file example_input.txt. To sort the nodes, run:

```
cat example_input.txt | ./wheelersort
```

This should print the Wheeler order of the nodes:

```
0 1 4 2 3 5
```