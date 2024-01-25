using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public static class AStar
{
    public static HashSet<Vector2Int> floorPositions;
    public static List<Vector2Int> FindPath(Vector2Int start, Vector2Int end)
    {
        // Create a list to store the path
        List<Vector2Int> path = new List<Vector2Int>();

        // Create a dictionary to store the cost to reach each position
        Dictionary<Vector2Int, float> costSoFar = new Dictionary<Vector2Int, float>();

        // Create a binary heap to store the positions to explore
        BinaryHeap<Vector2Int> frontier = new BinaryHeap<Vector2Int>();

        // Initialize the starting position
        costSoFar[start] = 0;
        frontier.Enqueue(start, 0);

        while (frontier.Count > 0)
        {
            // Get the position with the lowest cost
            Vector2Int current = frontier.Dequeue();

            // Check if we have reached the destination
            if (current == end)
            {
                break;
            }

            // Explore the neighbors of the current position
            foreach (Vector2Int next in GetNeighbors(current))
            {
                float newCost = costSoFar[current] + GetCost(current, next);

                if (!costSoFar.ContainsKey(next) || newCost < costSoFar[next])
                {
                    costSoFar[next] = newCost;
                    float priority = newCost + Heuristic(end, next);
                    frontier.Enqueue(next, priority);
                }
            }
        }

        // Reconstruct the path from start to end
        Vector2Int currentPos = end;
        while (currentPos != start)
        {
            path.Add(currentPos);
            currentPos = GetLowestCostNeighbor(currentPos, costSoFar);
        }
        path.Reverse();
        path.Add(end);

        return path;
    }

    public static List<Vector2Int> FindPathFromGrid(Vector2Int start, Vector2Int end)
    {
        //If floorpositions is not null and start or end is in floorpositions or a neighbour of floorpositions
        if (floorPositions != null &&  floorPositions.Contains(start) || floorPositions.Intersect(GetNeighbors(start)).Any() || floorPositions.Contains(end) || floorPositions.Intersect(GetNeighbors(end)).Any())
        {
            if (!floorPositions.Contains(start))
            {
                // Set the start node to the nearest floor position
                start = GetClosestFloorPosition(start);
            }

            // Check if the end node is outside of the floor positions
            if (!floorPositions.Contains(end))
            {
                // Set the end node to the nearest floor position
                end = GetClosestFloorPosition(end);
            }

            int maxIterations = 10000;
            int iterations = 0;



            // Create a dictionary to store the cost to reach each position
            Dictionary<Vector2Int, float> costSoFar = new Dictionary<Vector2Int, float>();

            //// Create a priority queue to store the positions to explore
            //PriorityQueue<Vector2Int> frontier = new PriorityQueue<Vector2Int>();
            // Create a binary heap to store the positions to explore
            BinaryHeap<Vector2Int> frontier = new BinaryHeap<Vector2Int>();

            // Initialize the starting position
            costSoFar[start] = 0;
            frontier.Enqueue(start, 0);

            while (frontier.Count > 0)
            {
                // Check if the maximum number of iterations has been exceeded
                if (iterations > maxIterations)
                {
                    Debug.Log("Maximum iterations exceeded. Breaking out of loop.");
                    break;
                }

                // Get the position with the lowest cost
                Vector2Int current = frontier.Dequeue();

                // Check if we have reached the destination
                if (current == end)
                {
                    break;
                }

                // Explore the neighbors of the current position
                foreach (Vector2Int next in GetNeighbors(current))
                {
                    // Only explore neighbors that are in the list of floor positions
                    if (!floorPositions.Contains(next))
                    {
                        continue;
                    }

                    float newCost = costSoFar[current] + GetCost(current, next);

                    if (!costSoFar.ContainsKey(next) || newCost < costSoFar[next])
                    {
                        costSoFar[next] = newCost;
                        float priority = newCost + Heuristic(end, next);
                        frontier.Enqueue(next, priority);
                    }
                }
                iterations++;
            }

            iterations = 0;
            // Reconstruct the path from start to end
            List<Vector2Int> path = new List<Vector2Int>();
            Vector2Int currentPos = end;
            while (currentPos != start)
            {
                path.Add(currentPos);
                currentPos = GetLowestCostNeighbor(currentPos, costSoFar);
                iterations++;
                // Check if the maximum number of iterations has been exceeded
                if (iterations > maxIterations)
                {
                    Debug.Log("Maximum iterations exceeded. Breaking out of loop.");
                    break;
                }
            }
            path.Reverse();
            path.Add(end);

            return path;
        }
        return null;
    }

    private static Vector2Int GetClosestFloorPosition(Vector2Int position)
    {
        Vector2Int closestFloorPosition = Vector2Int.zero;
        float closestDistance = float.MaxValue;

        foreach (Vector2Int floorPosition in floorPositions)
        {
            float distance = Vector2Int.Distance(position, floorPosition);

            if (distance < closestDistance)
            {
                closestFloorPosition = floorPosition;
                closestDistance = distance;
            }
        }

        return closestFloorPosition;
    }


    // Returns a list of neighboring positions for a given position
    private static List<Vector2Int> GetNeighbors(Vector2Int position)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>();
        neighbors.Add(position + Vector2Int.right); // Right neighbor
        neighbors.Add(position + Vector2Int.left); // Left neighbor
        neighbors.Add(position + Vector2Int.up); // Top neighbor
        neighbors.Add(position + Vector2Int.down); // Bottom neighbor
        return neighbors;
    }

    // Calculates the cost of moving from one position to another
    private static float GetCost(Vector2Int current, Vector2Int next)
    {
        return Vector2Int.Distance(current, next); // Distance between the two positions
    }

    // Calculates the heuristic cost of moving from one position to the goal
    private static float Heuristic(Vector2Int end, Vector2Int next)
    {
        return Vector2Int.Distance(end, next); // Distance between the current position and the goal
    }

    // Returns the neighbor with the lowest cost so far
    private static Vector2Int GetLowestCostNeighbor(Vector2Int position, Dictionary<Vector2Int, float> costSoFar)
    {
        List<Vector2Int> neighbors = GetNeighbors(position); // Get neighboring positions
        Vector2Int lowestCostNeighbor = neighbors[0]; // Assume the first neighbor has the lowest cost
        float lowestCost = float.MaxValue; // Assume the highest possible cost initially
        foreach (Vector2Int neighbor in neighbors)
        {
            if (costSoFar.ContainsKey(neighbor) && costSoFar[neighbor] < lowestCost)
            {
                lowestCostNeighbor = neighbor; // Update the lowest cost neighbor
                lowestCost = costSoFar[neighbor]; // Update the lowest cost
            }
        }
        return lowestCostNeighbor; // Return the neighbor with the lowest cost so far
    }
}


//public class PriorityQueue<T>
//{
//    private List<KeyValuePair<float, T>> queue = new List<KeyValuePair<float, T>>();

//    public int Count
//    {
//        get { return queue.Count; }
//    }

//    public void Enqueue(T item, float priority)
//    {
//        queue.Add(new KeyValuePair<float, T>(priority, item));
//        queue.Sort((a, b) => a.Key.CompareTo(b.Key));
//    }

//    public T Dequeue()
//    {
//        T item = queue[0].Value;
//        queue.RemoveAt(0);
//        return item;
//    }

public class BinaryHeap<T>
{
    private class HeapNode
    {
        public float priority;
        public T item;

        public HeapNode(float priority, T item)
        {
            this.priority = priority;
            this.item = item;
        }
    }

    private List<HeapNode> heap = new List<HeapNode>();

    public int Count
    {
        get { return heap.Count; }
    }

    public void Enqueue(T item, float priority)
    {
        heap.Add(new HeapNode(priority, item));
        int index = heap.Count - 1;
        while (index > 0)
        {
            int parentIndex = (index - 1) / 2;
            if (heap[index].priority < heap[parentIndex].priority)
            {
                SwapNodes(index, parentIndex);
                index = parentIndex;
            }
            else
            {
                break;
            }
        }
    }

    public T Dequeue()
    {
        T item = heap[0].item;
        int lastIndex = heap.Count - 1;
        heap[0] = heap[lastIndex];
        heap.RemoveAt(lastIndex);
        lastIndex--;

        int index = 0;
        while (true)
        {
            int leftChildIndex = index * 2 + 1;
            int rightChildIndex = index * 2 + 2;

            if (leftChildIndex > lastIndex)
            {
                break;
            }

            int minChildIndex = leftChildIndex;
            if (rightChildIndex <= lastIndex && heap[rightChildIndex].priority < heap[leftChildIndex].priority)
            {
                minChildIndex = rightChildIndex;
            }

            if (heap[minChildIndex].priority < heap[index].priority)
            {
                SwapNodes(minChildIndex, index);
                index = minChildIndex;
            }
            else
            {
                break;
            }
        }

        return item;
    }

    private void SwapNodes(int index1, int index2)
    {
        HeapNode temp = heap[index1];
        heap[index1] = heap[index2];
        heap[index2] = temp;
    }
}
