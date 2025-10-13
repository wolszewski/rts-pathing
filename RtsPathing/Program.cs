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
int unitCount = 300;
var units = new Unit[unitCount];
for (int i = 0; i < unitCount; i++)
{
    var p = new Vector2(rng.Next(-350, 351), rng.Next(-350, 351));
    var v = Vector2.Zero;
    units[i] = new Unit
    {
        Pos = p,
        Vel = v,
        Radius = 4f + (float)rng.NextDouble()*10f,
        Selected = false,
        HasTarget = false,
        Target = p,
        Facing = 0f,
        StuckTimer = 0f,
        LastDistToTarget = 0f
    };
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
            u.StuckTimer = 0f; // Reset stuck timer on new command
            u.LastDistToTarget = Vector2.Distance(u.Pos, clickWorld);
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

    // 1) Move towards targets with collision avoidance
    for (int i = 0; i < units.Length; i++)
    {
        var u = units[i];

        if (u.HasTarget)
        {
            Vector2 to = u.Target - u.Pos;
            float dist = to.Length();
            
            // Check if unit arrived at target
            if (dist <= GameConfig.ArriveDist)
            {
                u.HasTarget = false;
                u.Vel = Vector2.Zero;
                u.StuckTimer = 0f;
            }
            else
            {
                // Stuck detection: check if making progress
                float progressRate = (u.LastDistToTarget - dist) / dt; // units per second
                
                if (progressRate < GameConfig.StuckProgressThreshold)
                {
                    // Not making good progress, increment stuck timer
                    u.StuckTimer += dt;
                    
                    if (u.StuckTimer >= GameConfig.StuckTimeThreshold)
                    {
                        // Stuck too long, give up on target
                        u.HasTarget = false;
                        u.Vel = Vector2.Zero;
                        u.StuckTimer = 0f;
                    }
                }
                else
                {
                    // Making progress, reset stuck timer
                    u.StuckTimer = 0f;
                }
                
                u.LastDistToTarget = dist;
                
                // Only proceed with movement if still has target
                if (u.HasTarget)
                {
                    var desiredDir = to / dist;
                    
                    // Calculate avoidance force from nearby units
                    Vector2 avoidanceForce = Vector2.Zero;
                    for (int j = 0; j < units.Length; j++)
                    {
                        if (i == j) continue;
                        
                        var other = units[j];
                        Vector2 toOther = other.Pos - u.Pos;
                        float distToOther = toOther.Length();
                        float avoidRadius = u.Radius + other.Radius + GameConfig.AvoidanceRange;
                        
                        // Check if other unit is within avoidance range
                        if (distToOther < avoidRadius && distToOther > 0.001f)
                        {
                            // Calculate avoidance direction (away from other unit)
                            Vector2 avoidDir = -toOther / distToOther;
                            
                            // Stronger avoidance when closer
                            float strength = (1f - (distToOther / avoidRadius));
                            
                            // Only avoid stationary units (or slow moving ones)
                            // Moving units can navigate around each other
                            if (other.Vel.LengthSquared() < 10f * 10f) // slower than 10 units/sec
                            {
                                avoidanceForce += avoidDir * strength * GameConfig.AvoidanceStrength;
                            }
                        }
                    }
                    
                    // Combine desired direction with avoidance
                    Vector2 combinedDir = desiredDir + avoidanceForce * dt;
                    
                    // Normalize if we have a direction
                    if (combinedDir.LengthSquared() > 0.001f)
                    {
                        combinedDir = Vector2.Normalize(combinedDir);
                    }
                    else
                    {
                        combinedDir = desiredDir;
                    }
                    
                    float targetAngle = MathF.Atan2(combinedDir.Y, combinedDir.X);
                    
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
        }
        else
        {
            // no target, stay
            u.Vel = Vector2.Zero;
            u.StuckTimer = 0f;
        }

        // integrate
        u.Pos += u.Vel * dt;

        // crude world bounds
        const float B = 500f;
        if (u.Pos.X < -B || u.Pos.X > B) { u.Vel.X = 0; u.Pos.X = Math.Clamp(u.Pos.X, -B, B); u.HasTarget = false; }
        if (u.Pos.Y < -B || u.Pos.Y > B) { u.Vel.Y = 0; u.Pos.Y = Math.Clamp(u.Pos.Y, -B, B); u.HasTarget = false; }

        units[i] = u;
    }

    // 2) Unit-unit collision resolution - only push stationary units apart
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
                
                // Check if units are moving or stationary
                bool iMoving = ui.HasTarget && ui.Vel.LengthSquared() > 1f;
                bool jMoving = uj.HasTarget && uj.Vel.LengthSquared() > 1f;
                
                if (!iMoving && !jMoving)
                {
                    // Both stationary - push apart evenly
                    ui.Pos -= n * (penetration * 0.5f);
                    uj.Pos += n * (penetration * 0.5f);
                }
                else if (iMoving && !jMoving)
                {
                    // i is moving, j is stationary - only push i back
                    ui.Pos -= n * penetration;
                }
                else if (!iMoving && jMoving)
                {
                    // j is moving, i is stationary - only push j back
                    uj.Pos += n * penetration;
                }
                else
                {
                    // Both moving - push apart evenly (normal collision)
                    ui.Pos -= n * (penetration * 0.5f);
                    uj.Pos += n * (penetration * 0.5f);
                }
                
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