// .NET 8 top-level program
// NuGet: Raylib-cs (namespace Raylib_cs)

using System.Numerics;
using Raylib_cs;

public static class Renderer
{
    /// <summary>
    /// Calculates the visible world bounds based on camera settings.
    /// Returns (minX, maxX, minY, maxY) in world coordinates.
    /// </summary>
    private static (float minX, float maxX, float minY, float maxY) GetVisibleWorldBounds(Camera2D cam, int screenWidth, int screenHeight)
    {
        // Get the four corners of the screen in world space
        Vector2 topLeft = Raylib.GetScreenToWorld2D(new Vector2(0, 0), cam);
        Vector2 topRight = Raylib.GetScreenToWorld2D(new Vector2(screenWidth, 0), cam);
        Vector2 bottomLeft = Raylib.GetScreenToWorld2D(new Vector2(0, screenHeight), cam);
        Vector2 bottomRight = Raylib.GetScreenToWorld2D(new Vector2(screenWidth, screenHeight), cam);
        
        // Find the bounding box
        float minX = MathF.Min(MathF.Min(topLeft.X, topRight.X), MathF.Min(bottomLeft.X, bottomRight.X));
        float maxX = MathF.Max(MathF.Max(topLeft.X, topRight.X), MathF.Max(bottomLeft.X, bottomRight.X));
        float minY = MathF.Min(MathF.Min(topLeft.Y, topRight.Y), MathF.Min(bottomLeft.Y, bottomRight.Y));
        float maxY = MathF.Max(MathF.Max(topLeft.Y, topRight.Y), MathF.Max(bottomLeft.Y, bottomRight.Y));
        
        return (minX, maxX, minY, maxY);
    }
    
    /// <summary>
    /// Check if a point is within the visible bounds (with margin for partial visibility).
    /// </summary>
    private static bool IsPointVisible(Vector2 point, float margin, float minX, float maxX, float minY, float maxY)
    {
        return point.X >= minX - margin && point.X <= maxX + margin &&
               point.Y >= minY - margin && point.Y <= maxY + margin;
    }
    
    /// <summary>
    /// Check if a line segment intersects with the visible bounds.
    /// </summary>
    private static bool IsLineVisible(Vector2 a, Vector2 b, float margin, float minX, float maxX, float minY, float maxY)
    {
        // Check if either endpoint is visible
        if (IsPointVisible(a, margin, minX, maxX, minY, maxY) || 
            IsPointVisible(b, margin, minX, maxX, minY, maxY))
            return true;
        
        // Check if line intersects the visible rectangle
        // Simple conservative check: if bounding box of line overlaps viewport
        float lineMinX = MathF.Min(a.X, b.X);
        float lineMaxX = MathF.Max(a.X, b.X);
        float lineMinY = MathF.Min(a.Y, b.Y);
        float lineMaxY = MathF.Max(a.Y, b.Y);
        
        return !(lineMaxX < minX - margin || lineMinX > maxX + margin ||
                 lineMaxY < minY - margin || lineMinY > maxY + margin);
    }
    
    public static void DrawGrid(int halfCells, int cellSize, Camera2D cam)
    {
        var (minX, maxX, minY, maxY) = GetVisibleWorldBounds(cam, GameConfig.ScreenW, GameConfig.ScreenH);
        
        int ext = halfCells * cellSize;
        
        // Calculate visible range for vertical lines
        int startCol = Math.Max(-halfCells, (int)MathF.Floor(minX / cellSize));
        int endCol = Math.Min(halfCells, (int)MathF.Ceiling(maxX / cellSize));
        
        // Draw vertical grid lines (only visible ones)
        for (int i = startCol; i <= endCol; i++)
        {
            int x = i * cellSize;
            Raylib.DrawLine(x, -ext, x, ext, i == 0 ? Color.LightGray : ColorAlpha(Color.LightGray, 0.5f));
        }
        
        // Calculate visible range for horizontal lines
        int startRow = Math.Max(-halfCells, (int)MathF.Floor(minY / cellSize));
        int endRow = Math.Min(halfCells, (int)MathF.Ceiling(maxY / cellSize));
        
        // Draw horizontal grid lines (only visible ones)
        for (int j = startRow; j <= endRow; j++)
        {
            int y = j * cellSize;
            Raylib.DrawLine(-ext, y, ext, y, j == 0 ? Color.LightGray : ColorAlpha(Color.LightGray, 0.5f));
        }
    }

    public static void DrawSelectionBox(Vector2 selectStartScreen, Vector2 selectEndScreen)
    {
        float x0 = MathF.Min(selectStartScreen.X, selectEndScreen.X);
        float y0 = MathF.Min(selectStartScreen.Y, selectEndScreen.Y);
        float x1 = MathF.Max(selectStartScreen.X, selectEndScreen.X);
        float y1 = MathF.Max(selectStartScreen.Y, selectEndScreen.Y);
        var rect = new Rectangle(x0, y0, x1 - x0, y1 - y0);
        Raylib.DrawRectangleRec(rect, ColorAlpha(Color.Yellow, 0.25f));
        Raylib.DrawRectangleLinesEx(rect, 1.5f, Color.Yellow);
    }

    public static void DrawOverlay(float zoom, Unit[] units, long tickCounter, int visibleUnits = -1, int visibleObstacles = -1)
    {
        // UI overlay
        Raylib.DrawRectangle(10, 10, 470, 140, ColorAlpha(Color.Black, 0.45f));
        int y = 18;
        Raylib.DrawText("Controls:", 20, y, 18, Color.RayWhite); y += 22;
        Raylib.DrawText("WASD - pan   Q/E or wheel - zoom   R - reset view", 20, y, 16, Color.RayWhite); y += 20;
        Raylib.DrawText("LMB drag - select units   RMB - move selected to point", 20, y, 16, Color.RayWhite); y += 20;
        Raylib.DrawText($"Tick dt: {GameConfig.TickDt * 1000f:0.#} ms | Ticks: {tickCounter}", 20, y, 16, Color.RayWhite); y += 20;
        
        string unitInfo = visibleUnits >= 0 ? $"{visibleUnits}/{units.Length}" : $"{units.Length}";
        string obsInfo = visibleObstacles >= 0 ? $"   Visible Obstacles: {visibleObstacles}" : "";
        Raylib.DrawText($"FPS: {Raylib.GetFPS()}   Zoom: {zoom:0.00}   Units: {unitInfo}{obsInfo}", 20, y, 16, Color.RayWhite);
    }

    public static int DrawObstacles((Vector2 A, Vector2 B)[] obstacles, Camera2D cam)
    {
        var (minX, maxX, minY, maxY) = GetVisibleWorldBounds(cam, GameConfig.ScreenW, GameConfig.ScreenH);
        
        // Add margin for thick lines
        const float margin = 50f;
        int drawnCount = 0;
        
        // Draw obstacles only if visible
        foreach (var seg in obstacles)
        {
            if (IsLineVisible(seg.A, seg.B, margin, minX, maxX, minY, maxY))
            {
                Raylib.DrawLineEx(seg.A, seg.B, 3f, Color.Red);
                drawnCount++;
            }
        }
        
        return drawnCount;
    }

    public static int DrawUnits(Unit[] units, Camera2D cam)
    {
        var (minX, maxX, minY, maxY) = GetVisibleWorldBounds(cam, GameConfig.ScreenW, GameConfig.ScreenH);
        int drawnCount = 0;
        
        // First draw target lines (so they appear behind units) - only for visible units
        foreach (ref var u in units.AsSpan())
        {
            if (u.HasTarget)
            {
                // Check if unit or target is visible (or line crosses viewport)
                float maxRadius = MathF.Max(u.Radius, 20f); // Account for unit size and target line
                if (IsLineVisible(u.Pos, u.Target, maxRadius, minX, maxX, minY, maxY))
                {
                    DrawDashedLine(u.Pos, u.Target, 2f, 8f, 4f, Color.Yellow);
                }
            }
        }
        
        // Then draw units on top - only if visible
        foreach (ref var u in units.AsSpan())
        {
            // Check if unit is within visible bounds (with margin for unit radius)
            if (IsPointVisible(u.Pos, u.Radius * 2f, minX, maxX, minY, maxY))
            {
                DrawUnit(ref u);
                drawnCount++;
            }
        }
        
        return drawnCount;
    }

    private static void DrawUnit(ref Unit u)
    {
        Raylib.DrawCircleV(u.Pos, u.Radius, Color.SkyBlue);
        if (u.Selected)
        {
            Raylib.DrawCircleLines((int)u.Pos.X, (int)u.Pos.Y, u.Radius + 1.5f, Color.Yellow);
        }
        // Draw facing direction arrow
        Vector2 facingDir = new Vector2(MathF.Cos(u.Facing), MathF.Sin(u.Facing));
        Vector2 arrowTip = u.Pos + facingDir * (u.Radius * 1.5f);
        Raylib.DrawLineV(u.Pos, arrowTip, Color.Blue);

        // Draw velocity indicator (optional - shows if movement matches facing)
        if (u.Vel.LengthSquared() > 1e-6f)
        {
            Vector2 velDir = Vector2.Normalize(u.Vel);
            Vector2 velTip = u.Pos + velDir * (u.Radius * 1.2f);
            Raylib.DrawCircleV(velTip, 2f, Color.Green);
        }
    }
    
    /// <summary>
    /// Draws a dashed line between two points
    /// </summary>
    private static void DrawDashedLine(Vector2 start, Vector2 end, float thickness, float dashLength, float gapLength, Color color)
    {
        Vector2 direction = end - start;
        float totalLength = direction.Length();
        
        if (totalLength < 0.001f) return;
        
        Vector2 normalizedDir = direction / totalLength;
        float patternLength = dashLength + gapLength;
        
        float currentDist = 0f;
        while (currentDist < totalLength)
        {
            float dashEnd = MathF.Min(currentDist + dashLength, totalLength);
            Vector2 dashStart = start + normalizedDir * currentDist;
            Vector2 dashEndPos = start + normalizedDir * dashEnd;
            
            Raylib.DrawLineEx(dashStart, dashEndPos, thickness, color);
            
            currentDist += patternLength;
        }
    }

    private static Color ColorAlpha(Color c, float a) => new Color(c.R, c.G, c.B, (byte)(a * 255));
}
