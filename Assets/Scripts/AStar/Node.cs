﻿using UnityEngine;

namespace AStarPathfinding
{
    public class Node
    {
        public Vector3 position;
        public Node parent;

        public bool walkable;
        public int gridX, gridZ;
        public int gCost, hCost;

        public int fCost
        {
            get
            {
                return gCost + hCost;
            }
        }
        
        /// <summary>
        /// Constructor for Node
        /// </summary>
        /// <param name = "walkable"> Detects whether node is walkable </param>
        /// <param name = "position"> Point where node is located </param>
        /// <param name = "gridX"> X coordinate in 2D array </param>
        /// <param name = "gridZ"> Z coordinate in 2D array </param>
        
        public Node (bool walkable, Vector3 position, int gridX, int gridZ)
        {
            this.walkable = walkable;
            this.position = position;

            this.gridX = gridX;
            this.gridZ = gridZ;
        }
    }
}