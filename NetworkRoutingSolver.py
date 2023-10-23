#!/usr/bin/python3


from CS312Graph import *
import time
import math


class NetworkRoutingSolver:
    def __init__(self):
        self.dest = None
        self.dist = []
        self.prev = []

    def initializeNetwork(self, network):
        assert (type(network) == CS312Graph)
        self.network = network

    def getShortestPath(self, destIndex):
        self.dest = destIndex

        path_edges = []

        curr_edge = self.prev[self.dest]
        total_cost = 0
        while curr_edge:
            path_edges.append((curr_edge.src.loc, curr_edge.dest.loc, '{:.0f}'.format(curr_edge.length)))
            total_cost += curr_edge.length
            curr_edge = self.prev[curr_edge.src.node_id]
        print("End of total cost function: Path edges: " + str(path_edges))
        print("End of total cost function: Total cost: " + str(total_cost))
        return {'cost': total_cost, 'path': path_edges}

    def computeShortestPaths(self, srcIndex, use_heap=False):
        self.source = srcIndex
        t1 = time.time()
        self.dist.clear()
        self.prev.clear()

        #Dijkstra's Algorithm
        if use_heap:
            queue = BinaryHeapPQ()
        else:
            queue = ArrayPQ()

        self.dist = [math.inf] * len(self.network.nodes)
        self.prev = [None] * len(self.network.nodes)
        self.dist[srcIndex] = 0

        queue.makeQueue(len(self.network.nodes), srcIndex)

        while queue.notEmpty():
            curr_node = queue.deleteMin(self.dist)
            curr_edges = self.network.nodes[curr_node].neighbors
            for i in range(len(curr_edges)):
                dest_node = curr_edges[i].dest.node_id

                if self.dist[curr_node] + curr_edges[i].length < self.dist[dest_node]:
                    self.dist[dest_node] = self.dist[curr_node] + curr_edges[i].length
                    self.prev[dest_node] = curr_edges[i]
                    queue.decreaseKey(dest_node, self.dist)

        t2 = time.time()
        return t2 - t1


class NodesQueue:
    def __init__(self):
        # implement queue using a list
        self.queue = []

    def __len__(self):
        return len(self.queue)

    def deleteMin(self, dist):
        pass

    def decreaseKey(self, index, dist):
        pass

    def insert(self, index, dist):
        pass

    def makeQueue(self, node_count, src_index):
        pass


class BinaryHeapPQ(NodesQueue):
    def __init__(self):
        super().__init__()
        # keeps track of where to find nodes in queue
        self.map = []

    def notEmpty(self):
        return len(self.queue) > 0

    def makeQueue(self, node_count, src_index):
        self.queue = [src_index]
        self.map = []
        map_add = 1
        for i in range(node_count):
            if i != src_index:
                self.queue.append(i)
                self.map.append(i + map_add)
            else:
                self.map.append(0)
                map_add = 0
        print("TESTING HEAP ARRAYS")
        print(self.queue)
        print(self.map)

    def deleteMin(self, dist):
        if not self.queue:
            return None
        if len(self.queue) == 1:
            return self.queue.pop()
        # store lowest value
        min_val = self.queue[0]
        self.map[min_val] = -1
        # moving node from bottom of heap to the top
        self.queue[0] = self.queue.pop()
        #update index in map
        self.map[self.queue[0]] = 0
        #resort the nodes to be in correct order
        self.sort_down(0, dist)
        return min_val

    def decreaseKey(self, node, dist):
        #finding the index of node
        index = self.map[node]
        if index == 0:
            return
        curr_index = index
        parent_index = self.get_parent_index(curr_index)
        while curr_index > 0 and dist[self.queue[parent_index]] > dist[self.queue[curr_index]]:
            self.swap(parent_index, curr_index)
            curr_index = parent_index
            parent_index = self.get_parent_index(curr_index)


    def sort_up(self, index, dist):
        parent = self.get_parent_index(index)
        #if the parent is greater than the new node, swap nodes
        if dist[self.queue[parent]] > dist[self.queue[index]]:
            self.swap(index, parent)
            #recursively move up until min value is at top
            self.sort_up(parent, dist)

    def sort_down(self, index, dist):
        min_child = self.get_min_child(index, dist)
        while min_child > 0 and dist[self.queue[index]] > dist[self.queue[min_child]]:
            self.swap(index, min_child)
            index = min_child
            min_child = self.get_min_child(index, dist)

    def swap(self, index1, index2):
        temp = self.queue[index1]
        self.queue[index1] = self.queue[index2]
        self.queue[index2] = temp

        temp = self.map[self.queue[index1]]
        self.map[self.queue[index1]] = self.map[self.queue[index2]]
        self.map[self.queue[index2]] = temp

    def get_parent_index(self, child):
        if child == 0:
            return None
        if child <= 2:
            return 0
        return (child - 1) // 2

    def get_left_child(self, parent):
        return (parent * 2) + 1

    def get_right_child(self, parent):
        return (parent * 2) + 2

    def get_last_index(self):
        return len(self.queue) - 1

    def get_min_child(self, parent, dist):
        left_child = self.get_left_child(parent)
        right_child = self.get_right_child(parent)

        #if there are no children
        if left_child > self.get_last_index():
            return -1
        if right_child > self.get_last_index():
            return left_child

        if dist[self.queue[left_child]] < dist[self.queue[right_child]]:
            return left_child
        return right_child


class ArrayPQ:
    def __init__(self):
        self.queue = []

    def insert(self, node):
        self.queue.append(node)

    def makeQueue(self, node_count, src_index):
        self.queue = [src_index]
        for i in range(node_count):
            if i != src_index:
                self.insert(i)

    def decreaseKey(self, node_id, dist):
        pass

    def deleteMin(self, dist):
        min_idx = 0
        for i in range(len(self.queue)):
            if dist[self.queue[i]] < dist[self.queue[min_idx]]:  # Compare values in dist that exist in queue
                min_idx = i
        min_val = self.queue[min_idx]
        del self.queue[min_idx]
        return min_val

    def notEmpty(self):
        return len(self.queue) > 0

