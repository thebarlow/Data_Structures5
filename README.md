# Data_Structures5

PROMPT
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
a. Implement the method computeAllEuclideanDistances() which should compute the euclidean distance between all cities that have a direct link 
and set the weights for the corresponding edges in the graph. Once this works correctly, the correct distances should be displayed in the
GUI when clicking on "Compute All Euclidean Distances".

b. In the method doDijkstra(String s), implement Dijkstra's algorithm starting at the city with name s. Use the distances associated with the edges. 
The method should update the distance and prev instance variables of the Vertex objects. You do not have to use a priority queue to store vertices that 
still need to be visited. Instead, keep these vertices on a list and scan through the entire list to find the minimum. We are making this simplification 
(at the expense of runtime) because java.util.PriorityQueue does not support the decreaseKey operation.

c. Implement the method getDijkstraPath(String s, String t), which first calls doDijstra(s) and then uses the distance and prev instance variables 
of the Vertex objects to find the shortest path between s and t. The resulting path should be returned as a list of Edge objects. Once this works 
correctly, you should be able to compute and display paths in the GUI.
