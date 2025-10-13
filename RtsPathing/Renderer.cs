// .NET 8 top-level program
// NuGet: Raylib-cs (namespace Raylib_cs)

using System.Numerics;
using Raylib_cs;

public static class Renderer
{
    public static void DrawGrid(int halfCells, int cellSize)
    {
        int ext = halfCells * cellSize;
        for (int i = -halfCells; i <= halfCells; i++)
        {
            int x = i * cellSize;
            Raylib.DrawLine(x, -ext, x, ext, i == 0 ? Color.LightGray : ColorAlpha(Color.LightGray, 0.5f));
        }
        for (int j = -halfCells; j <= halfCells; j++)
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

    public static void DrawOverlay(float zoom, Unit[] units, long tickCounter)
    {
        // UI overlay
        Raylib.DrawRectangle(10, 10, 470, 120, ColorAlpha(Color.Black, 0.45f));
        int y = 18;
        Raylib.DrawText("Controls:", 20, y, 18, Color.RayWhite); y += 22;
        Raylib.DrawText("WASD - pan   Q/E or wheel - zoom   R - reset view", 20, y, 16, Color.RayWhite); y += 20;
        Raylib.DrawText("LMB drag - select units   RMB - move selected to point", 20, y, 16, Color.RayWhite); y += 20;
        Raylib.DrawText($"Tick dt: {GameConfig.TickDt * 1000f:0.#} ms (200 Hz) | Ticks: {tickCounter}", 20, y, 16, Color.RayWhite); y += 20;
        Raylib.DrawText($"FPS: {Raylib.GetFPS()}   Zoom: {zoom:0.00}   Units: {units.Length}", 20, y, 16, Color.RayWhite);
    }

    public static void DrawObstacles((Vector2 A, Vector2 B)[] obstacles)
    {
        // Draw obstacles
        foreach (var seg in obstacles)
            Raylib.DrawLineEx(seg.A, seg.B, 3f, Color.Red);
    }

    public static void DrawUnits(Unit[] units)
    {
        // Draw units
        foreach (ref var u in units.AsSpan())
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
    }

    private static Color ColorAlpha(Color c, float a) => new Color(c.R, c.G, c.B, (byte)(a * 255));
}
