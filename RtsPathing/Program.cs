// .NET 8 top-level program
// NuGet: Raylib-cs (namespace Raylib_cs)

using System.Numerics;
using Raylib_cs;


// Camera params
float zoom = 1.0f;
var cam = new Camera2D
{
    Target = Vector2.Zero,
    Offset = new Vector2(GameConfig.ScreenW / 2f, GameConfig.ScreenH / 2f),
    Rotation = 0f,
    Zoom = zoom
};

// Obstacles: line segments
var obstacles = new (Vector2 A, Vector2 B)[] {
    (new Vector2(-200, -50), new Vector2(200, -50)),
    (new Vector2(-300, 100), new Vector2(-50, 300)),
    (new Vector2(50, 100), new Vector2(300, 300)),
    (new Vector2(-100, -200), new Vector2(100, -300)),
};

// Units: simple circles with positions/vels (placeholder for your pathing)
var rng = new Random(42);
int unitCount = 40;
var units = new Unit[unitCount];
for (int i = 0; i < unitCount; i++)
{
    var p = new Vector2(rng.Next(-350, 351), rng.Next(-350, 351));
    var v = Vector2.Zero;
    units[i] = new Unit { Pos = p, Vel = v, Radius = 8f, Selected = false, HasTarget = false, Target = p, Facing = 0f };
}

// Init window
Raylib.SetConfigFlags(ConfigFlags.VSyncHint); // keep smooth; remove if you want uncapped
Raylib.InitWindow(GameConfig.ScreenW, GameConfig.ScreenH, "2D RTS Pathing – Rendering Scaffold (raylib-cs)");
Raylib.SetTargetFPS(GameConfig.TargetFps);

// Fixed timestep accumulator
double accumulator = 0.0;
double currentTime = Raylib.GetTime();
long tickCounter = 0;

// Input settings
float camPanSpeed = 500f;  // world units / second at zoom=1
float zoomSpeed = 1.0f;    // scroll/keys

// Selection state (screen-space while dragging)
bool isSelecting = false;
Vector2 selectStartScreen = Vector2.Zero;
Vector2 selectEndScreen = Vector2.Zero;
const float ClickDragThreshold = 5f; // pixels

while (!Raylib.WindowShouldClose())
{
    // ----------------------------
    // Time step management
    // ----------------------------
    double newTime = Raylib.GetTime();
    double frameTime = newTime - currentTime;
    currentTime = newTime;
    accumulator += frameTime;

    // ----------------------------
    // Handle input (camera)
    // ----------------------------
    var worldPan = camPanSpeed * (float)frameTime / cam.Zoom;

    if (Raylib.IsKeyDown(KeyboardKey.W)) cam.Target.Y -= worldPan;
    if (Raylib.IsKeyDown(KeyboardKey.S)) cam.Target.Y += worldPan;
    if (Raylib.IsKeyDown(KeyboardKey.A)) cam.Target.X -= worldPan;
    if (Raylib.IsKeyDown(KeyboardKey.D)) cam.Target.X += worldPan;

    if (Raylib.IsKeyDown(KeyboardKey.Q)) zoom *= (1f + zoomSpeed * (float)frameTime);
    if (Raylib.IsKeyDown(KeyboardKey.E)) zoom *= (1f - zoomSpeed * (float)frameTime);
    zoom = Math.Clamp(zoom, 0.25f, 4f);
    cam.Zoom = zoom;

    if (Raylib.IsKeyPressed(KeyboardKey.R))
    {
        cam.Target = Vector2.Zero;
        cam.Zoom = zoom = 1f;
    }

    // Optional: mouse wheel zoom centered at mouse cursor
    float mw = Raylib.GetMouseWheelMove();
    if (Math.Abs(mw) > 0.0001f)
    {
        var mouseScreen = Raylib.GetMousePosition();
        var before = Raylib.GetScreenToWorld2D(mouseScreen, cam);
        zoom *= (mw > 0 ? 1.1f : 0.9f);
        zoom = Math.Clamp(zoom, 0.25f, 4f);
        cam.Zoom = zoom;
        var after = Raylib.GetScreenToWorld2D(mouseScreen, cam);
        cam.Target += before - after; // keep point under cursor stable
    }

    // ----------------------------
    // Mouse selection input
    // ----------------------------
    if (Raylib.IsMouseButtonPressed(MouseButton.Left))
    {
        isSelecting = true;
        selectStartScreen = Raylib.GetMousePosition();
        selectEndScreen = selectStartScreen;
    }

    if (isSelecting)
    {
        selectEndScreen = Raylib.GetMousePosition();
    }

    if (isSelecting && Raylib.IsMouseButtonReleased(MouseButton.Left))
    {
        var drag = selectEndScreen - selectStartScreen;
        if (drag.LengthSquared() <= ClickDragThreshold * ClickDragThreshold)
        {
            // Treat as click selection: pick unit under cursor
            var clickWorld = Raylib.GetScreenToWorld2D(selectEndScreen, cam);
            int bestIdx = -1;
            float bestDistSq = float.MaxValue;
            for (int i = 0; i < units.Length; i++)
            {
                var u = units[i];
                float r = u.Radius;
                float d2 = Vector2.DistanceSquared(u.Pos, clickWorld);
                if (d2 <= r * r && d2 < bestDistSq)
                {
                    bestDistSq = d2;
                    bestIdx = i;
                }
            }

            if (bestIdx >= 0)
            {
                // exclusive selection
                for (int i = 0; i < units.Length; i++)
                {
                    var u = units[i];
                    u.Selected = (i == bestIdx);
                    units[i] = u;
                }
            }
            // If no unit hit, leave selection unchanged
        }
        else
        {
            // Build world-space AABB from dragged screen rectangle
            var ss0 = selectStartScreen;
            var ss1 = selectEndScreen;
            Vector2 w0 = Raylib.GetScreenToWorld2D(ss0, cam);
            Vector2 w1 = Raylib.GetScreenToWorld2D(ss1, cam);
            float minX = MathF.Min(w0.X, w1.X);
            float maxX = MathF.Max(w0.X, w1.X);
            float minY = MathF.Min(w0.Y, w1.Y);
            float maxY = MathF.Max(w0.Y, w1.Y);

            // Clear existing selection and select units with center inside the AABB
            for (int i = 0; i < units.Length; i++)
            {
                var u = units[i];
                u.Selected = u.Pos.X >= minX && u.Pos.X <= maxX && u.Pos.Y >= minY && u.Pos.Y <= maxY;
                units[i] = u;
            }
        }

        isSelecting = false;
    }

    // Issue move command to selected units with RMB
    if (Raylib.IsMouseButtonPressed(MouseButton.Right))
    {
        var clickWorld = Raylib.GetScreenToWorld2D(Raylib.GetMousePosition(), cam);
        for (int i = 0; i < units.Length; i++)
        {
            if (!units[i].Selected) continue;
            var u = units[i];
            u.Target = clickWorld;
            u.HasTarget = true;
            units[i] = u;
        }
    }

    // ----------------------------
    // Fixed update loop (5ms)
    // ----------------------------
    while (accumulator >= GameConfig.TickDt)
    {
        FixedUpdate((float)GameConfig.TickDt, units, obstacles);
        accumulator -= GameConfig.TickDt;
        tickCounter++;
    }

    // ----------------------------
    // Render
    // ----------------------------
    Raylib.BeginDrawing();
    Raylib.ClearBackground(Color.DarkGray);

    Raylib.BeginMode2D(cam);

    Renderer.DrawGrid(50, 50);
    Renderer.DrawObstacles(obstacles);
    Renderer.DrawUnits(units);
    Raylib.EndMode2D();

    // Selection rectangle (screen-space overlay)
    if (isSelecting)
    {
        Renderer.DrawSelectionBox(selectStartScreen, selectEndScreen);
    }

    Renderer.DrawOverlay(zoom, units, tickCounter);

    Raylib.EndDrawing();
}

// Cleanup
Raylib.CloseWindow();


static void FixedUpdate(float dt, Unit[] units, (Vector2 A, Vector2 B)[] obstacles)
{
    float sharpTurnThreshold = (180f - GameConfig.SharpTurnAngleDeg) * MathF.PI / 180f;
    // If angle difference is greater than this, unit stops to rotate in place

    // 1) Move towards targets
    for (int i = 0; i < units.Length; i++)
    {
        var u = units[i];

        if (u.HasTarget)
        {
            Vector2 to = u.Target - u.Pos;
            float dist = to.Length();
            if (dist <= GameConfig.ArriveDist)
            {
                u.HasTarget = false;
                u.Vel = Vector2.Zero;
            }
            else
            {
                var desiredDir = to / dist;
                float targetAngle = MathF.Atan2(desiredDir.Y, desiredDir.X);
                
                // Calculate initial angle difference to determine if sharp turn is needed
                float initialAngleDiff = NormalizeAngle(targetAngle - u.Facing);
                
                // Check if we need a sharp turn BEFORE rotating
                bool isSharpTurn = MathF.Abs(initialAngleDiff) > sharpTurnThreshold;
                
                // Smooth rotation towards target angle
                float maxRotation = GameConfig.RotationSpeed * dt;
                
                if (MathF.Abs(initialAngleDiff) <= maxRotation)
                {
                    // Close enough, snap to target angle
                    u.Facing = targetAngle;
                }
                else
                {
                    // Rotate by max amount in the correct direction
                    u.Facing += MathF.Sign(initialAngleDiff) * maxRotation;
                }
                
                // Normalize facing angle to [-PI, PI]
                u.Facing = NormalizeAngle(u.Facing);
                
                // Apply velocity based on whether it's a sharp turn
                if (isSharpTurn)
                {
                    // Sharp turn required - stop and rotate in place
                    u.Vel = Vector2.Zero;
                }
                else
                {
                    // Can move while rotating
                    Vector2 currentDir = new Vector2(MathF.Cos(u.Facing), MathF.Sin(u.Facing));
                    u.Vel = currentDir * GameConfig.MaxSpeed;
                }
            }
        }
        else
        {
            // no target, stay
            u.Vel = Vector2.Zero;
        }

        // integrate
        u.Pos += u.Vel * dt;

        // crude world bounds
        const float B = 500f;
        if (u.Pos.X < -B || u.Pos.X > B) { u.Vel.X = 0; u.Pos.X = Math.Clamp(u.Pos.X, -B, B); u.HasTarget = false; }
        if (u.Pos.Y < -B || u.Pos.Y > B) { u.Vel.Y = 0; u.Pos.Y = Math.Clamp(u.Pos.Y, -B, B); u.HasTarget = false; }

        units[i] = u;
    }

    // 2) Unit-unit collision detection and resolution (stop on collision)
    for (int i = 0; i < units.Length; i++)
    {
        for (int j = i + 1; j < units.Length; j++)
        {
            var ui = units[i];
            var uj = units[j];
            Vector2 d = uj.Pos - ui.Pos;
            float distSq = d.LengthSquared();
            float minDist = ui.Radius + uj.Radius;
            float minDistSq = minDist * minDist;
            if (distSq < minDistSq)
            {
                float dist = MathF.Sqrt(MathF.Max(distSq, 1e-6f));
                Vector2 n = dist > 1e-5f ? d / dist : new Vector2(1f, 0f);
                float penetration = minDist - dist;
                // push apart evenly
                ui.Pos -= n * (penetration * 0.5f);
                uj.Pos += n * (penetration * 0.5f);
                // stop both and clear targets
                ui.Vel = Vector2.Zero; ui.HasTarget = false;
                uj.Vel = Vector2.Zero; uj.HasTarget = false;
                units[i] = ui;
                units[j] = uj;
            }
        }
    }
}

// Helper to normalize angle to [-PI, PI] range
static float NormalizeAngle(float angle)
{
    while (angle > MathF.PI) angle -= MathF.PI * 2;
    while (angle < -MathF.PI) angle += MathF.PI * 2;
    return angle;
}

public static class GameClock
{

}