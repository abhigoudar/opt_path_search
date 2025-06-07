# Time-optimal path planning

## Description

This repository contains a path planning algorithm based on Dijkstra's method. 

#### Objective: Calculate the shortest time path between given start and end states.

The problem of time-optimal path planning can be cast a search problem on a weighted graph where
* The states are vertices in a graph,
* the possible actions are edges between the vertices,
* the weights (cost) on the edges are times associated with taking the action associated with the edge.

Finding the time-optimal path is then similar to finding the least cost path on the graph (see Section 2.3.1 in Reference [1]).

The Dijkstras algorithm presented in this repoistory makes the following assumptions about the data being provided:

* There exists atleast one path from the start state to the end state.
* The time associated each action is non-negative (>= 0). 
* The total cost associated with the optimal path can be accomodated in a `uint32_t` data type.

The above assumptions can be relaxed easily.

## Overview of the algorithm

The general flow of the algorithm is as follows:

* Step 1: Begin at the start node.
* Step 2: For each node calculate the time to go each of its neighbour nodes in the graph.
* Step 3: Update the cost of each neighbour with the time to go if the current time to go is larger. Also, store the name of the neighbour with the smallest time.
* Step 4: Push the neighbours on a priority queue, where neighbours are sorted based on the time to reach them.
* Step 5: Select the neighbour with the small time (or cost-to-go).
* Step 6: If end node reached, terminate else go to Step 1.

The optimal path can then be read from the queue that stores the lowest cost neighbours.

More details on the algorithm can be found at the following reference.

## References

LaValle, S. M. (2006). Planning algorithms. _Planning Algorithms_, _9780521862_, 1–826. [https://doi.org/10.1017/CBO9780511546877](https://doi.org/10.1017/CBO9780511546877)

## Compiling and Running code

### Installing dependendcies

NOTE: The package has been developed and tested in a Linux-based (Ubuntu 24.04) operating system. Support for Windows and MacOS does not exist.

The main requirement is `cmake`, which can be installed as follows

```bash
sudo apt install cmake
```

### Clone the repository

```bash
git clone https://github.com/abhigoudar/opt_path_search.git
```

### Build

The following instructions assume that the repository was cloned in the `/home/$USER` directory. If the repository is cloned in a custom location, please execute the commands in the repository's main directory.

```bash
cd ~/opt_path_search
mkdir build && cd build
cmake ..
make
```

This should produce a binary file called `plan_shortest_time` in the `build` directory.

### Running the algorithm

The repoistory contains representative examples in the `data` directory. The command line instruction for testing testing the algorithm is as follows:

```bash
./plan_shortest_time <problem JSON file> <states JSON file> <actions JSON file>
```

where 

* `<problem JSON file>` is a JSON file containing the start and goal states. 

* `<states JSON file>` is a JSON file containing all the possible states. 

* `<actions JSON file>` is a JSON file containing all the actions, the corresponding start and end states, and the time associated with taking each action.


The exact format can be found in `problem_dummy` directory.


Example: 
```c++
cd ~/opt_path_search/build
./plan_shortest_time ../data/problem_dummy/problem.json ../data/problem_dummy/states.json ../data/problem_dummy/actions.json
```

This should produce the following output

```bash

 Done calculating shortest path 

-->state1-->state2-->state3


 Optimal action list (end to start) 

-->action1-->action3

 ----------- Total time of shortest path: 11

 Writing optimal action list to: ../data/problem_dummy/optimal_actions.json

```

As noted in the above snippet, running the algorithm will produce a `optimal_actions.json` file containing the list of actions to take from `start_state` to `end_state` that incur the least time, which is also included in the output JSON file.

