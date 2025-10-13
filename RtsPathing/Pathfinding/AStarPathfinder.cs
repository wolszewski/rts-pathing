using System.Numerics;

namespace RtsPathing.Pathfinding;

/// <summary>
/// A* pathfinding implementation with optimizations for RTS games.
/// Features:
/// - Efficient priority queue
/// - 8-directional movement
/// - Straight-line optimization
/// - Group path caching
/// </summary>
public class AStarPathfinder : IPathfinder
{
    private readonly PathfindingGrid _grid;
    private readonly PriorityQueue<(int x, int y), float> _openSet;
    private readonly Dictionary<(int, int), float> _gScore;
    private readonly Dictionary<(int, int), float> _fScore;
    private readonly Dictionary<(int, int), (int, int)> _cameFrom;
    private readonly HashSet<(int, int)> _closedSet;
    
    // 8-directional movement offsets
    private static readonly (int dx, int dy)[] Neighbors =
    {
        (1, 0), (-1, 0), (0, 1), (0, -1),      // Cardinal
        (1, 1), (1, -1), (-1, 1), (-1, -1)     // Diagonal
    };
    
    public AStarPathfinder(PathfindingGrid grid)
    {
        _grid = grid;
        _openSet = new PriorityQueue<(int x, int y), float>();
        _gScore = new Dictionary<(int, int), float>();
        _fScore = new Dictionary<(int, int), float>();
        _cameFrom = new Dictionary<(int, int), (int, int)>();
        _closedSet = new HashSet<(int, int)>();
    }
    
    public List<Vector2>? FindPath(Vector2 start, Vector2 goal)
    {
        var (startX, startY) = _grid.WorldToGrid(start);
        var (goalX, goalY) = _grid.WorldToGrid(goal);
        
        // If start and goal are the same, return single point
        if (startX == goalX && startY == goalY)
            return new List<Vector2> { start };
        
        // Check if start or goal are blocked - only allow small adjustments
        bool startBlocked = !_grid.IsWalkable(startX, startY);
        bool goalBlocked = !_grid.IsWalkable(goalX, goalY);
        
        if (startBlocked)
        {
            // Try to find a very close walkable cell (1 cell radius only)
            var nearbyStart = FindNearestWalkableCell(startX, startY, searchRadius: 1);
            if (nearbyStart.HasValue)
            {
                (startX, startY) = nearbyStart.Value;
            }
            else
            {
                // Start is blocked and no immediate alternative - just use direct path
                return new List<Vector2> { start, goal };
            }
        }
        
        if (goalBlocked)
        {
            // Try to find a very close walkable cell (1 cell radius only)  
            var nearbyGoal = FindNearestWalkableCell(goalX, goalY, searchRadius: 1);
            if (nearbyGoal.HasValue)
            {
                (goalX, goalY) = nearbyGoal.Value;
            }
            else
            {
                // Goal is blocked and no immediate alternative - just use direct path
                return new List<Vector2> { start, goal };
            }
        }
        
        // Clear previous search data
        _openSet.Clear();
        _gScore.Clear();
        _fScore.Clear();
        _cameFrom.Clear();
        _closedSet.Clear();
        
        // Initialize start node
        var startNode = (startX, startY);
        var goalNode = (goalX, goalY);
        
        _gScore[startNode] = 0;
        _fScore[startNode] = Heuristic(startNode, goalNode);
        _openSet.Enqueue(startNode, _fScore[startNode]);
        
        // A* main loop
        while (_openSet.Count > 0)
        {
            var current = _openSet.Dequeue();
            
            // Skip if already processed (closed set prevents reprocessing)
            if (_closedSet.Contains(current))
                continue;
            
            // Mark as processed
            _closedSet.Add(current);
            
            // Goal reached
            if (current == goalNode)
            {
                return ReconstructPath(current, start, goal);
            }
            
            // Explore neighbors
            foreach (var (dx, dy) in Neighbors)
            {
                int neighborX = current.x + dx;
                int neighborY = current.y + dy;
                var neighbor = (neighborX, neighborY);
                
                // Skip if out of bounds or blocked
                if (!_grid.IsWalkable(neighborX, neighborY))
                    continue;
                
                // Skip if already processed
                if (_closedSet.Contains(neighbor))
                    continue;
                
                // Calculate movement cost (diagonal = ?2 ? 1.414)
                float moveCost = (dx != 0 && dy != 0) ? 1.414f : 1.0f;
                float tentativeGScore = _gScore[current] + moveCost;
                
                // Check if this path is better
                if (!_gScore.TryGetValue(neighbor, out float neighborGScore) || tentativeGScore < neighborGScore)
                {
                    _cameFrom[neighbor] = current;
                    _gScore[neighbor] = tentativeGScore;
                    _fScore[neighbor] = tentativeGScore + Heuristic(neighbor, goalNode);
                    
                    _openSet.Enqueue(neighbor, _fScore[neighbor]);
                }
            }
        }
        
        // No path found - return direct path as fallback
        return new List<Vector2> { start, goal };
    }
    
    public void UpdateObstacles(IEnumerable<(Vector2 A, Vector2 B)> obstacles)
    {
        // Clear existing obstacles
        _grid.ClearAllBlocked();
        
        // Mark all obstacles as blocked with moderate thickness
        // Cell size is 50 units, thickness of 1 gives 3-cell wide blockage = 150 units
        // This provides clearance while not being too aggressive
        foreach (var obstacle in obstacles)
        {
            _grid.MarkLineAsBlocked(obstacle.A, obstacle.B, thickness: 1);
        }
    }
    
    /// <summary>
    /// Heuristic function for A* (Euclidean distance).
    /// </summary>
    private float Heuristic((int x, int y) from, (int x, int y) to)
    {
        int dx = to.x - from.x;
        int dy = to.y - from.y;
        return MathF.Sqrt(dx * dx + dy * dy);
    }
    
    /// <summary>
    /// Reconstruct path from goal to start, then smooth it.
    /// </summary>
    private List<Vector2> ReconstructPath((int x, int y) current, Vector2 worldStart, Vector2 worldGoal)
    {
        var pathGrid = new List<(int x, int y)> { current };
        
        // Trace back through cameFrom
        while (_cameFrom.TryGetValue(current, out var previous))
        {
            pathGrid.Add(previous);
            current = previous;
        }
        
        // Reverse to get start -> goal order
        pathGrid.Reverse();
        
        // Convert to world coordinates
        var pathWorld = new List<Vector2>();
        foreach (var node in pathGrid)
        {
            pathWorld.Add(_grid.GridToWorld(node.x, node.y));
        }
        
        // Smooth the path (remove unnecessary waypoints)
        var smoothed = SmoothPath(pathWorld);
        
        // Ensure exact start and goal positions
        if (smoothed.Count > 0)
        {
            smoothed[0] = worldStart;
            smoothed[^1] = worldGoal;
        }
        
        return smoothed;
    }
    
    /// <summary>
    /// Remove unnecessary waypoints using line-of-sight checks.
    /// </summary>
    private List<Vector2> SmoothPath(List<Vector2> path)
    {
        if (path.Count <= 2)
            return path;
        
        var smoothed = new List<Vector2> { path[0] };
        int current = 0;
        
        while (current < path.Count - 1)
        {
            int farthest = current + 1;
            
            // Find farthest visible point
            for (int i = current + 2; i < path.Count; i++)
            {
                if (HasLineOfSight(path[current], path[i]))
                {
                    farthest = i;
                }
                else
                {
                    break; // No point checking further
                }
            }
            
            smoothed.Add(path[farthest]);
            current = farthest;
        }
        
        return smoothed;
    }
    
    /// <summary>
    /// Check if there's a clear line of sight between two points.
    /// </summary>
    private bool HasLineOfSight(Vector2 from, Vector2 to)
    {
        var (x0, y0) = _grid.WorldToGrid(from);
        var (x1, y1) = _grid.WorldToGrid(to);
        
        // Bresenham's line algorithm
        int dx = Math.Abs(x1 - x0);
        int dy = Math.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        
        while (true)
        {
            // Check if current cell is blocked
            if (!_grid.IsWalkable(x0, y0))
                return false;
            
            if (x0 == x1 && y0 == y1)
                return true;
            
            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }
    
    /// <summary>
    /// Find the nearest walkable cell to the given position.
    /// </summary>
    private (int x, int y)? FindNearestWalkableCell(int x, int y, int searchRadius)
    {
        // First check if current position is walkable
        if (_grid.IsWalkable(x, y))
            return (x, y);
        
        // Check in expanding squares, prioritizing closer cells
        List<(int x, int y, float dist)> candidates = new();
        
        for (int radius = 1; radius <= searchRadius; radius++)
        {
            for (int dx = -radius; dx <= radius; dx++)
            {
                for (int dy = -radius; dy <= radius; dy++)
                {
                    // Only check cells on the perimeter of the current radius
                    if (Math.Abs(dx) == radius || Math.Abs(dy) == radius)
                    {
                        int checkX = x + dx;
                        int checkY = y + dy;
                        if (_grid.IsWalkable(checkX, checkY))
                        {
                            float dist = MathF.Sqrt(dx * dx + dy * dy);
                            candidates.Add((checkX, checkY, dist));
                        }
                    }
                }
            }
            
            // If we found candidates at this radius, return the closest one
            if (candidates.Count > 0)
            {
                candidates.Sort((a, b) => a.dist.CompareTo(b.dist));
                return (candidates[0].x, candidates[0].y);
            }
        }
        
        return null;
    }
}
