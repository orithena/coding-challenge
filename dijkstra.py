#!/usr/bin/python3

"""
dijkstra.py -- Implementation of the get-in-it coding challenge 2019

Call as:
    python3 dijkstra.py generatedGraph.json Erde b3-r7-r4nd7


COPYRIGHT 2019 David Kliczbor <maligree@gmx.de>

The following license applies ONLY AFTER 2019-06-07 (i.e. when the coding 
challenge posted at https://www.get-in-it.de/coding-challenge is over)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
"""

import sys
import json
from heapq import heapify, heappop
from functools import total_ordering

def find(seq, condition):
    return [n for n in seq if condition(n)][0]


class Edge:
    def __init__(self, source, target, cost):
        self.source = source
        self.target = target
        self.cost = cost
        
    def __repr__(self):
        return f"{self.target.label}:{self.cost:#6.4f}"

@total_ordering
class Node:
    def __init__(self, label):
        self.label = label
        self.adjacent = list()
        self.cost = float('inf')
        self.predecessor = None
    def __lt__(self, other):
        return self.cost < other.cost
    def __eq__(self, other):
        return self.cost == other.cost
        
    def add_adjacent(self, target, cost):
        self.adjacent.append(Edge(self, target, cost))
        
    def return_path(self):
        path = list()
        node = self
        if node.predecessor is not None:
            while node is not None:
                path.append(node)
                node = node.predecessor
        else:
            raise KeyError()
        return path
            
    def __str__(self):
        return f"{self.label}: {', '.join([str(e) for e in self.adjacent])}"
    def __repr__(self):
        return f"{self.label}: {self.cost:#6.4f}"

class Graph:
    def __init__(self, filename):
        self.nodes = list()
        data = json.load(open(filename))
        for node in data.get("nodes"):
            self.add_node(**node)
        for edge in data.get("edges"):
            self.add_edge(**edge)
    
    def add_node(self, label):
        self.nodes.append(Node(label))
    
    def add_edge(self, source, target, cost):
        self.nodes[source].add_adjacent(self.nodes[target], cost)
        self.nodes[target].add_adjacent(self.nodes[source], cost)
        
    def dijkstra(self, source_label, target_label):
        # reset costs
        for node in self.nodes:
            node.cost = float('inf')
            node.predecessor = None
        # find source and target nodes by label
        source = find(self.nodes, lambda n: n.label == source_label)
        target = find(self.nodes, lambda n: n.label == target_label)
        # cost to travel from source to source is 0
        source.cost = 0
        # prepare priority queue
        queue = self.nodes.copy()
        heapify(queue)
        # run the queue
        while len(queue) > 0:
            # pop node with lowest cost 
            # (made possible by functools.total_ordering, Node.__lt__ and Node.__eq__)
            node = heappop(queue)
            if node == target:
                # hooray, we have found the target on shortest path!
                break
            # check all adjacent edges
            for outedge in node.adjacent:
                # only consider nodes that are still in queue
                if outedge.target in queue:
                    # check the cumulative cost to the target of the current edge
                    cost = node.cost + outedge.cost
                    if cost < outedge.target.cost:
                        # cumulative cost to reach target of current edge 
                        # is lower than on any other path found until now
                        # sooo ... we set the new, lower cost to reach the edge target
                        outedge.target.cost = cost
                        # ... and mark our way back to the last node 
                        # (needed for backtracing in Node.return_path)
                        outedge.target.predecessor = node
                        # rebuild the heap, since we just changed a cost
                        heapify(queue)
        # calculate return path
        path = target.return_path()
        # and return minimal cost and path
        return path[0].cost, list(reversed(path))
        
    def __str__(self):
        return "\n".join([str(n) for n in self.nodes])

if __name__ == "__main__":
    try:
        _, jsonfile, source, target, *_ = sys.argv
        g = Graph(jsonfile)
        cost, path = g.dijkstra(source, target)
        print(f"Travelling from {source} to {target} costs {cost} Galacticoins. Here's the path and the cumulated cost to each stop:")
        print(" -> ".join(f"{n.label} ({n.cost:#6.4f} GC)" for n in path))
    except ValueError:
        print("Not enough arguments. Use: python3 dijkstra.py generatedGraph.json Erde b3-r7-r4nd7")
    except IndexError:
        print(f"'{source}' or '{target}' is not in the set of known stops")
    except FileNotFoundError:
        print(f"Cannot open input file '{jsonfile}'")
    except KeyError:
        print(f"There is no path from {source} to {target}")
        