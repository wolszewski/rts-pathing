using System.Numerics;

namespace RtsPathing.Pathfinding;

/// <summary>
/// Grid representation for pathfinding.
/// Converts world coordinates to grid cells and tracks walkability.
/// </summary>
public class PathfindingGrid
{
    private readonly bool[,] _walkable; // true = walkable, false = blocked
    private readonly int _width;
    private readonly int _height;
    private readonly int _cellSize;
    
    public int Width => _width;
    public int Height => _height;
    public int CellSize => _cellSize;
    
    public PathfindingGrid(int width, int height, int cellSize)
    {
        _width = width;
        _height = height;
        _cellSize = cellSize;
        _walkable = new bool[width, height];
        
        // Initialize all cells as walkable
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                _walkable[x, y] = true;
            }
        }
    }
    
    /// <summary>
    /// Convert world position to grid coordinates.
    /// </summary>
    public (int x, int y) WorldToGrid(Vector2 worldPos)
    {
        float worldMinX = -_width * _cellSize * 0.5f;
        float worldMinY = -_height * _cellSize * 0.5f;
        
        int gridX = (int)((worldPos.X - worldMinX) / _cellSize);
        int gridY = (int)((worldPos.Y - worldMinY) / _cellSize);
        
        return (gridX, gridY);
    }
    
    /// <summary>
    /// Convert grid coordinates to world position (center of cell).
    /// </summary>
    public Vector2 GridToWorld(int gridX, int gridY)
    {
        float worldMinX = -_width * _cellSize * 0.5f;
        float worldMinY = -_height * _cellSize * 0.5f;
        
        float worldX = worldMinX + (gridX + 0.5f) * _cellSize;
        float worldY = worldMinY + (gridY + 0.5f) * _cellSize;
        
        return new Vector2(worldX, worldY);
    }
    
    /// <summary>
    /// Check if grid coordinates are within bounds.
    /// </summary>
    public bool IsInBounds(int x, int y)
    {
        return x >= 0 && x < _width && y >= 0 && y < _height;
    }
    
    /// <summary>
    /// Check if a cell is walkable.
    /// </summary>
    public bool IsWalkable(int x, int y)
    {
        if (!IsInBounds(x, y)) return false;
        return _walkable[x, y];
    }
    
    /// <summary>
    /// Set a cell's walkability.
    /// </summary>
    public void SetWalkable(int x, int y, bool walkable)
    {
        if (IsInBounds(x, y))
        {
            _walkable[x, y] = walkable;
        }
    }
    
    /// <summary>
    /// Mark all cells intersecting a line segment as unwalkable.
    /// Uses Bresenham's line algorithm.
    /// </summary>
    public void MarkLineAsBlocked(Vector2 start, Vector2 end, int thickness = 1)
    {
        var (x0, y0) = WorldToGrid(start);
        var (x1, y1) = WorldToGrid(end);
        
        // Bresenham's line algorithm
        int dx = Math.Abs(x1 - x0);
        int dy = Math.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        
        while (true)
        {
            // Mark cell and surrounding cells based on thickness
            for (int tx = -thickness; tx <= thickness; tx++)
            {
                for (int ty = -thickness; ty <= thickness; ty++)
                {
                    SetWalkable(x0 + tx, y0 + ty, false);
                }
            }
            
            if (x0 == x1 && y0 == y1) break;
            
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
    /// Mark all cells inside a circle as unwalkable.
    /// </summary>
    public void MarkCircleAsBlocked(Vector2 center, float radius)
    {
        var (cx, cy) = WorldToGrid(center);
        int gridRadius = (int)Math.Ceiling(radius / _cellSize);
        
        for (int dx = -gridRadius; dx <= gridRadius; dx++)
        {
            for (int dy = -gridRadius; dy <= gridRadius; dy++)
            {
                if (dx * dx + dy * dy <= gridRadius * gridRadius)
                {
                    SetWalkable(cx + dx, cy + dy, false);
                }
            }
        }
    }
    
    /// <summary>
    /// Mark all cells inside a rectangle as unwalkable.
    /// </summary>
    public void MarkRectangleAsBlocked(Vector2 min, Vector2 max)
    {
        var (minX, minY) = WorldToGrid(min);
        var (maxX, maxY) = WorldToGrid(max);
        
        for (int x = minX; x <= maxX; x++)
        {
            for (int y = minY; y <= maxY; y++)
            {
                SetWalkable(x, y, false);
            }
        }
    }
    
    /// <summary>
    /// Clear all blocked cells (make entire grid walkable).
    /// </summary>
    public void ClearAllBlocked()
    {
        for (int x = 0; x < _width; x++)
        {
            for (int y = 0; y < _height; y++)
            {
                _walkable[x, y] = true;
            }
        }
    }
}
