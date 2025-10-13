using System.Numerics;

/// <summary>
/// Responsible for creating and configuring game maps with obstacles and unit placements.
/// Provides predefined map layouts and procedural generation capabilities.
/// </summary>
public static class MapBuilder
{
    /// <summary>
    /// Creates the default map with rectangles, circles, and corridors.
    /// </summary>
    public static (List<(Vector2 A, Vector2 B)> obstacles, Vector2[] unitPositions) BuildDefaultMap(int unitCount, int seed = 42)
    {
        var obstacles = new List<(Vector2 A, Vector2 B)>();
        
        // Outer boundary walls
        AddBoundaryWalls(obstacles);
        
        // Large central rectangle (400x400)
        AddRectangle(obstacles, new Vector2(-200, -200), new Vector2(200, 200));
        
        // Corner rectangles
        AddRectangle(obstacles, new Vector2(-1800, -1500), new Vector2(-1200, -1300)); // Top-left
        AddRectangle(obstacles, new Vector2(1200, -1500), new Vector2(1800, -1300));   // Top-right
        AddRectangle(obstacles, new Vector2(-1700, 1200), new Vector2(-1450, 1500));   // Bottom-left
        AddRectangle(obstacles, new Vector2(1450, 1200), new Vector2(1700, 1500));     // Bottom-right
        
        // Vertical corridors (creates passage areas)
        AddVerticalLine(obstacles, -800, -1000, 1000);
        AddVerticalLine(obstacles, -600, -1000, 1000);
        AddVerticalLine(obstacles, 600, -1000, 1000);
        AddVerticalLine(obstacles, 800, -1000, 1000);
        
        // Small scattered square obstacles
        AddRectangle(obstacles, new Vector2(-400, -800), new Vector2(-300, -700));  // Top-left area
        AddRectangle(obstacles, new Vector2(300, -800), new Vector2(400, -700));    // Top-right area
        AddRectangle(obstacles, new Vector2(-1000, 400), new Vector2(-900, 500));   // Mid-left
        AddRectangle(obstacles, new Vector2(900, 400), new Vector2(1000, 500));     // Mid-right
        AddRectangle(obstacles, new Vector2(-500, 800), new Vector2(-400, 900));    // Bottom-left
        AddRectangle(obstacles, new Vector2(400, 800), new Vector2(500, 900));      // Bottom-right
        
        // Circular obstacles at strategic positions
        AddCircleObstacle(obstacles, new Vector2(0, -1000), 150, 20);      // Top center
        AddCircleObstacle(obstacles, new Vector2(0, 1000), 150, 20);       // Bottom center
        AddCircleObstacle(obstacles, new Vector2(-1200, 0), 120, 18);      // Left center
        AddCircleObstacle(obstacles, new Vector2(1200, 0), 120, 18);       // Right center
        AddCircleObstacle(obstacles, new Vector2(-1500, 700), 100, 16);    // Bottom-left
        AddCircleObstacle(obstacles, new Vector2(1500, 700), 100, 16);     // Bottom-right
        AddCircleObstacle(obstacles, new Vector2(-1500, -700), 100, 16);   // Top-left
        AddCircleObstacle(obstacles, new Vector2(1500, -700), 100, 16);    // Top-right
        
        // Generate random unit positions
        var unitPositions = GenerateRandomUnitPositions(unitCount, seed);
        
        return (obstacles, unitPositions);
    }
    
    /// <summary>
    /// Creates a simple test map with minimal obstacles for testing basic pathfinding.
    /// </summary>
    public static (List<(Vector2 A, Vector2 B)> obstacles, Vector2[] unitPositions) BuildTestMap(int unitCount, int seed = 42)
    {
        var obstacles = new List<(Vector2 A, Vector2 B)>();
        
        // Simple boundary
        AddBoundaryWalls(obstacles);
        
        // Single central obstacle
        AddRectangle(obstacles, new Vector2(-300, -300), new Vector2(300, 300));
        
        // Two circular obstacles
        AddCircleObstacle(obstacles, new Vector2(-1000, 0), 200, 20);
        AddCircleObstacle(obstacles, new Vector2(1000, 0), 200, 20);
        
        var unitPositions = GenerateRandomUnitPositions(unitCount, seed);
        
        return (obstacles, unitPositions);
    }
    
    /// <summary>
    /// Creates a maze-like map with corridors and dead ends.
    /// </summary>
    public static (List<(Vector2 A, Vector2 B)> obstacles, Vector2[] unitPositions) BuildMazeMap(int unitCount, int seed = 42)
    {
        var obstacles = new List<(Vector2 A, Vector2 B)>();
        
        AddBoundaryWalls(obstacles);
        
        // Horizontal maze walls
        for (int i = -4; i <= 4; i++)
        {
            int y = i * 400;
            if (i % 2 == 0)
            {
                AddHorizontalLine(obstacles, y, -2000, -200);
                AddHorizontalLine(obstacles, y, 200, 2000);
            }
            else
            {
                AddHorizontalLine(obstacles, y, -2000, 2000);
            }
        }
        
        // Vertical maze walls
        for (int i = -4; i <= 4; i++)
        {
            int x = i * 400;
            if (i % 2 != 0)
            {
                AddVerticalLine(obstacles, x, -2000, -200);
                AddVerticalLine(obstacles, x, 200, 2000);
            }
        }
        
        var unitPositions = GenerateRandomUnitPositions(unitCount, seed, -1800, 1800);
        
        return (obstacles, unitPositions);
    }
    
    /// <summary>
    /// Creates an open map with scattered circular obstacles (good for testing formation movement).
    /// </summary>
    public static (List<(Vector2 A, Vector2 B)> obstacles, Vector2[] unitPositions) BuildOpenMap(int unitCount, int seed = 42)
    {
        var obstacles = new List<(Vector2 A, Vector2 B)>();
        
        AddBoundaryWalls(obstacles);
        
        // Scattered circular obstacles
        var rng = new Random(seed + 1000);
        for (int i = 0; i < 20; i++)
        {
            float x = rng.Next(-2000, 2001);
            float y = rng.Next(-2000, 2001);
            float radius = rng.Next(80, 151);
            AddCircleObstacle(obstacles, new Vector2(x, y), radius, 16);
        }
        
        var unitPositions = GenerateRandomUnitPositions(unitCount, seed);
        
        return (obstacles, unitPositions);
    }
    
    // Helper methods for adding obstacles
    
    private static void AddBoundaryWalls(List<(Vector2 A, Vector2 B)> obstacles)
    {
        float bound = 2500;
        // Top
        obstacles.Add((new Vector2(-bound, -2000), new Vector2(bound, -2000)));
        // Bottom
        obstacles.Add((new Vector2(-bound, 2000), new Vector2(bound, 2000)));
        // Left
        obstacles.Add((new Vector2(-bound, -2000), new Vector2(-bound, 2000)));
        // Right
        obstacles.Add((new Vector2(bound, -2000), new Vector2(bound, 2000)));
    }
    
    private static void AddRectangle(List<(Vector2 A, Vector2 B)> obstacles, Vector2 min, Vector2 max)
    {
        // Top
        obstacles.Add((new Vector2(min.X, min.Y), new Vector2(max.X, min.Y)));
        // Right
        obstacles.Add((new Vector2(max.X, min.Y), new Vector2(max.X, max.Y)));
        // Bottom
        obstacles.Add((new Vector2(max.X, max.Y), new Vector2(min.X, max.Y)));
        // Left
        obstacles.Add((new Vector2(min.X, max.Y), new Vector2(min.X, min.Y)));
    }
    
    private static void AddHorizontalLine(List<(Vector2 A, Vector2 B)> obstacles, float y, float xStart, float xEnd)
    {
        obstacles.Add((new Vector2(xStart, y), new Vector2(xEnd, y)));
    }
    
    private static void AddVerticalLine(List<(Vector2 A, Vector2 B)> obstacles, float x, float yStart, float yEnd)
    {
        obstacles.Add((new Vector2(x, yStart), new Vector2(x, yEnd)));
    }
    
    private static void AddCircleObstacle(List<(Vector2 A, Vector2 B)> obstacles, Vector2 center, float radius, int segments)
    {
        for (int i = 0; i < segments; i++)
        {
            float angle1 = (i / (float)segments) * MathF.PI * 2f;
            float angle2 = ((i + 1) / (float)segments) * MathF.PI * 2f;
            
            Vector2 p1 = center + new Vector2(MathF.Cos(angle1), MathF.Sin(angle1)) * radius;
            Vector2 p2 = center + new Vector2(MathF.Cos(angle2), MathF.Sin(angle2)) * radius;
            
            obstacles.Add((p1, p2));
        }
    }
    
    private static Vector2[] GenerateRandomUnitPositions(int count, int seed, int minRange = -2000, int maxRange = 2000)
    {
        var rng = new Random(seed);
        var positions = new Vector2[count];
        
        for (int i = 0; i < count; i++)
        {
            positions[i] = new Vector2(
                rng.Next(minRange, maxRange + 1),
                rng.Next(minRange, maxRange + 1)
            );
        }
        
        return positions;
    }
}
