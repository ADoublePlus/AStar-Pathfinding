using System.Collections.Generic;
using UnityEngine;

namespace AStarPathfinding
{
    public class AIAgent : MonoBehaviour
    {
        public List<Node> path;
        public Transform target;

        public float speed = 20f;
        public float stoppingDistance = 0;

        private Graph graph;

        private float remainingDistance = 0;

        // Use this for initialization
        void Start ()
        {
            // Find 'graph' in the scene
            graph = FindObjectOfType<Graph>();

            // Check for errors
            if (graph == null)
            {
                // Log an error
                Debug.LogError("Error: There is no generated graph in the scene!");

                Debug.Break();
            }
        }

        // Update is called once per frame
        void Update ()
        {
            // Get remainingDistance to target
            remainingDistance = Vector3.Distance(transform.position, target.position);

            // Check if remainingDistance is greater than the stoppingDistance
            if (remainingDistance >= stoppingDistance)
            {
                // Calculate the path
                path = RunAStar(transform.position, target.position);

                graph.path = path;

                // Check if there are nodes in the path
                if (path.Count > 0)
                {
                    // Get the next node
                    Node current = path[0];

                    // Move towards the next node
                    transform.position = Vector3.MoveTowards(transform.position, current.position, speed * Time.deltaTime);
                }
            }
        }

        public List<Node> RunAStar (Vector3 startPos, Vector3 targetPos)
        {
            // The set of nodes to be evaluated
            List<Node> openList = new List<Node>();

            // The set of nodes that are already evaluated
            List<Node> closedList = new List<Node>();

            // Get the startNode and targetNode
            Node startNode = graph.GetNodeFromPosition(startPos);
            Node targetNode = graph.GetNodeFromPosition(targetPos);

            // Add the start node to openList
            openList.Add(startNode);

            // While not all nodes have been checked
            while (openList.Count > 0)
            {
                // Set currentNode to node in openList with lowest fCost
                Node currentNode = FindShortestNode(openList);

                // Remove current node from openList
                openList.Remove(currentNode);

                // Add currentNode to closedList
                closedList.Add(currentNode);

                // Check if the currentNode is our targetNode
                if (currentNode == targetNode)
                {
                    // Lead the path you must!
                    path = RetracePath(startNode, targetNode);

                    // Return the path
                    return path;
                }

                // Loop through each neighbour of the current node
                foreach (Node neighbour in graph.GetNeighbours(currentNode))
                {
                    // Check if the neighbour is not walkable or neighbour is in the closedList
                    if (!neighbour.walkable || closedList.Contains(neighbour))
                    {
                        continue;
                    }

                    // Calculate cost to neighbour
                    int newCostToNeighbour = currentNode.gCost + GetHeuristic(currentNode, neighbour);

                    // Check if new cost is less than neighbour's gCost or if the openList does NOT contain neighbour
                    if (newCostToNeighbour < neighbour.gCost || !openList.Contains(neighbour))
                    {
                        // Set the neighbour's gCost to the new neighbour's gCost
                        neighbour.gCost = newCostToNeighbour;

                        // Calculate heuristic by getting distance from neighbour to targetNode
                        neighbour.hCost = GetHeuristic(neighbour, targetNode);

                        // Set neighbour's parent to currentNode
                        neighbour.parent = currentNode;

                        // Check if the neighbour is not in the openList
                        if (!openList.Contains(neighbour))
                        {
                            // Add neighbour to openlist
                            openList.Add(neighbour);
                        }
                    }
                }
            }

            return path;
        }

        int GetHeuristic (Node nodeA, Node nodeB)
        {
            // Get distance between nodeA index and nodeB index
            //int dstX = NewMethod(nodeA, nodeB);
            int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
            int dstZ = Mathf.Abs(nodeA.gridZ - nodeB.gridZ);

            // Find the greatest value and return appropriate formula for heuristic
            // 14 x min + 10 x (max - min)
            if (dstX > dstZ)
                return 14 * dstZ + 10 * (dstX - dstZ);

            return 14 * dstX + 10 * (dstZ - dstX);
        }

        /*private static int NewMethod (Node nodeA, Node nodeB)
        {
            return Mathf.Abs(nodeA.gridX - nodeB.gridX);
        }*/

        Node FindShortestNode(List<Node> nodeList)
        {
            Node shortestNode = null;

            float minFCost = float.MaxValue;
            float minHCost = float.MaxValue;

            // Loop through node list
            for (int i = 0; i < nodeList.Count; i++)
            {
                // Get currentNode
                Node currentNode = nodeList[i];

                // Check if the costs of node, less than min costs
                if (currentNode.fCost <= minFCost && currentNode.hCost <= minHCost)
                {
                    // Set the shortestNode and cost to currentNode's
                    minFCost = currentNode.fCost;
                    minHCost = currentNode.hCost;

                    shortestNode = currentNode;
                }
            }

            return shortestNode;
        }

        List<Node> RetracePath (Node start, Node end)
        {
            // Create a list to store the path
            List<Node> path = new List<Node>();

            // Set the currentNode to the end node
            Node currentNode = end;

            // Retrace the path back to the start node
            while (currentNode != start)
            {
                // Add the currentNode to the path
                path.Add(currentNode);

                // Traverse up the parent tree
                currentNode = currentNode.parent;
            }

            // The path is in reverse order so reverse path before returning
            path.Reverse();

            return path;
        }
    }
}