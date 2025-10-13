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
int unitCount = 50;
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
        LastDistToTarget = 0f,
        GroupId = 0,  // 0 means no group
        RestPosition = p,  // Initial rest position
        WasPushed = false,
        TotalPushDistance = 0f,
        PushRecoveryTimer = 0f
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
        
        // Collect selected unit indices
        var selectedIndices = new List<int>();
        for (int i = 0; i < units.Length; i++)
        {
            if (units[i].Selected)
                selectedIndices.Add(i);
        }
        
        if (selectedIndices.Count > 0)
        {
            // Calculate formation positions around the clicked point (now size-aware)
            Vector2[] formationPositions;
            if (GameConfig.UseCircularFormation)
            {
                formationPositions = FormationCalculator.CalculateCircularFormation(
                    clickWorld, 
                    selectedIndices.Count, 
                    GameConfig.FormationSpacing,
                    units,  // Pass unit array for size information
                    selectedIndices.ToArray());  // Pass selected indices
            }
            else
            {
                formationPositions = FormationCalculator.CalculateGridFormation(
                    clickWorld, 
                    selectedIndices.Count, 
                    GameConfig.FormationSpacing,
                    units,  // Pass unit array for size information
                    selectedIndices.ToArray());  // Pass selected indices
            }
            
            // Assign formation positions to units
            FormationCalculator.AssignFormationPositions(units, selectedIndices.ToArray(), formationPositions);
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

    // 1) Move towards targets with collision avoidance (or return to rest position if pushed)
    for (int i = 0; i < units.Length; i++)
    {
        var u = units[i];

        // Check if unit was pushed and needs to return to rest position
        if (u.WasPushed && !u.HasTarget)
        {
            Vector2 toRest = u.RestPosition - u.Pos;
            float distToRest = toRest.Length();
            
            // Increment recovery timer
            u.PushRecoveryTimer += dt;
            
            // Give up if taking too long to return (prevents infinite circling)
            if (u.PushRecoveryTimer > 3.0f) // 3 seconds timeout
            {
                u.WasPushed = false;
                u.Vel = Vector2.Zero;
                u.TotalPushDistance = 0f;
                u.PushRecoveryTimer = 0f;
                u.RestPosition = u.Pos; // Accept new position as rest position
            }
            else
            {
                // Use larger threshold if being pushed by nearby units (prevents circling at rest position)
                float arriveThreshold = GameConfig.ArriveDist;
                if (distToRest < 15f) // Near rest position
                {
                    // Count how many units are very close
                    int nearbyCount = 0;
                    for (int k = 0; k < units.Length; k++)
                    {
                        if (k == i) continue;
                        float nearDist = Vector2.Distance(u.Pos, units[k].Pos);
                        if (nearDist < u.Radius + units[k].Radius + 5f)
                            nearbyCount++;
                    }
                    // If crowded, accept arrival easier
                    if (nearbyCount >= 2)
                        arriveThreshold = GameConfig.ArriveDist * 2.5f;
                }
                
                if (distToRest <= arriveThreshold)
                {
                    // Arrived back at rest position
                    u.WasPushed = false;
                    u.Vel = Vector2.Zero;
                    u.TotalPushDistance = 0f;
                    u.PushRecoveryTimer = 0f;
                }
                else
                {
                    // Move back to rest position at reduced speed
                    var dir = toRest / distToRest;
                    float targetAngle = MathF.Atan2(dir.Y, dir.X);
                    
                    // Smooth rotation towards target angle
                    float initialAngleDiff = NormalizeAngle(targetAngle - u.Facing);
                    float maxRotation = GameConfig.RotationSpeed * dt;
                    
                    if (MathF.Abs(initialAngleDiff) <= maxRotation)
                    {
                        u.Facing = targetAngle;
                    }
                    else
                    {
                        u.Facing += MathF.Sign(initialAngleDiff) * maxRotation;
                    }
                    
                    u.Facing = NormalizeAngle(u.Facing);
                    
                    // Move at reduced speed toward rest position
                    // Slow down as we approach to reduce overshoot
                    float speedScale = distToRest > 20f ? 1f : Math.Max(0.4f, distToRest / 20f);
                    Vector2 currentDir = new Vector2(MathF.Cos(u.Facing), MathF.Sin(u.Facing));
                    u.Vel = currentDir * GameConfig.ReturnToRestSpeed * speedScale;
                }
            }
        }
        else if (u.HasTarget)
        {
            Vector2 to = u.Target - u.Pos;
            float dist = to.Length();
            
            // Check if unit arrived at target
            // Use larger threshold if being pushed by nearby units (prevents circling at destination)
            float arriveThreshold = GameConfig.ArriveDist;
            if (dist < 15f) // Near destination
            {
                // Count how many units are very close
                int nearbyCount = 0;
                for (int k = 0; k < units.Length; k++)
                {
                    if (k == i) continue;
                    float nearDist = Vector2.Distance(u.Pos, units[k].Pos);
                    if (nearDist < u.Radius + units[k].Radius + 5f)
                        nearbyCount++;
                }
                // If crowded, accept arrival easier
                if (nearbyCount >= 2)
                    arriveThreshold = GameConfig.ArriveDist * 2.5f;
            }
            
            if (dist <= arriveThreshold)
            {
                u.HasTarget = false;
                u.Vel = Vector2.Zero;
                u.StuckTimer = 0f;
                // Update rest position to new location
                u.RestPosition = u.Pos;
                u.WasPushed = false;
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
                        // Update rest position when giving up
                        u.RestPosition = u.Pos;
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
                    
                    // Calculate how close we are to target (0 = far, 1 = arrived)
                    float arrivalFactor = 1f - Math.Clamp((dist - GameConfig.ArriveDist) / 50f, 0f, 1f);
                    
                    // Calculate avoidance force from nearby units
                    Vector2 avoidanceForce = Vector2.Zero;
                    
                    // Only calculate avoidance if not very close to target (prevents circling at destination)
                    if (dist > GameConfig.ArriveDist * 2f)
                    {
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
                                    // Reduce avoidance strength when near target to prevent circling
                                    float avoidanceScale = (1f - arrivalFactor * 0.8f);
                                    avoidanceForce += avoidDir * strength * GameConfig.AvoidanceStrength * avoidanceScale;
                                }
                            }
                        }
                    }
                    
                    // Combine desired direction with avoidance
                    // Weight avoidance less when we're very close to target
                    Vector2 combinedDir = desiredDir + avoidanceForce * (1f - arrivalFactor * 0.9f);
                    
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
                        // Slow down as we approach target
                        float speedScale = dist > 30f ? 1f : Math.Max(0.3f, dist / 30f);
                        Vector2 currentDir = new Vector2(MathF.Cos(u.Facing), MathF.Sin(u.Facing));
                        u.Vel = currentDir * GameConfig.MaxSpeed * speedScale;
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

    // 2) Unit-unit collision resolution with group awareness and push recovery
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
                bool iMoving = (ui.HasTarget || ui.WasPushed) && ui.Vel.LengthSquared() > 1f;
                bool jMoving = (uj.HasTarget || uj.WasPushed) && uj.Vel.LengthSquared() > 1f;
                
                // Check if units are in the same group
                bool sameGroup = ui.GroupId > 0 && ui.GroupId == uj.GroupId;
                
                if (!iMoving && !jMoving)
                {
                    // Both stationary - push apart evenly
                    ui.Pos -= n * (penetration * 0.5f);
                    uj.Pos += n * (penetration * 0.5f);
                }
                else if (iMoving && !jMoving)
                {
                    // i is moving, j is stationary
                    if (sameGroup)
                    {
                        // Same group: push and wake up stationary unit
                        // Use asymmetric push - moving unit pushes harder
                        float movingPush = penetration * (1f - GameConfig.MovingUnitPushRatio);
                        float stationaryPush = penetration * GameConfig.MovingUnitPushRatio;
                        
                        ui.Pos -= n * movingPush;  // Moving unit pushed back less
                        uj.Pos += n * stationaryPush;  // Stationary unit pushed more
                        
                        if (!uj.HasTarget && Vector2.Distance(uj.Pos, uj.Target) > GameConfig.ArriveDist)
                        {
                            uj.HasTarget = true;
                            uj.StuckTimer = 0f;
                            uj.LastDistToTarget = Vector2.Distance(uj.Pos, uj.Target);
                            uj.WasPushed = false; // Clear pushed flag when reactivating
                            uj.TotalPushDistance = 0f;
                        }
                    }
                    else
                    {
                        // Different groups: moving unit pushes harder than stationary
                        float movingPush = penetration * (1f - GameConfig.MovingUnitPushRatio);
                        float stationaryPush = penetration * GameConfig.MovingUnitPushRatio;
                        
                        ui.Pos -= n * movingPush;  // Moving unit pushed back less
                        uj.Pos += n * stationaryPush;  // Stationary unit pushed more
                        uj.TotalPushDistance += stationaryPush;
                        
                        // Mark unit as pushed if accumulated displacement is significant
                        if (uj.TotalPushDistance > GameConfig.PushDistanceThreshold && !uj.WasPushed)
                        {
                            uj.WasPushed = true;
                        }
                    }
                }
                else if (!iMoving && jMoving)
                {
                    // j is moving, i is stationary
                    if (sameGroup)
                    {
                        // Same group: push and wake up stationary unit
                        // Use asymmetric push - moving unit pushes harder
                        float movingPush = penetration * (1f - GameConfig.MovingUnitPushRatio);
                        float stationaryPush = penetration * GameConfig.MovingUnitPushRatio;
                        
                        ui.Pos -= n * stationaryPush;  // Stationary unit pushed more
                        uj.Pos += n * movingPush;  // Moving unit pushed back less
                        
                        if (!ui.HasTarget && Vector2.Distance(ui.Pos, ui.Target) > GameConfig.ArriveDist)
                        {
                            ui.HasTarget = true;
                            ui.StuckTimer = 0f;
                            ui.LastDistToTarget = Vector2.Distance(ui.Pos, ui.Target);
                            ui.WasPushed = false; // Clear pushed flag when reactivating
                            ui.TotalPushDistance = 0f;
                        }
                    }
                    else
                    {
                        // Different groups: moving unit pushes harder than stationary
                        float movingPush = penetration * (1f - GameConfig.MovingUnitPushRatio);
                        float stationaryPush = penetration * GameConfig.MovingUnitPushRatio;
                        
                        ui.Pos -= n * stationaryPush;  // Stationary unit pushed more
                        uj.Pos += n * movingPush;  // Moving unit pushed back less
                        ui.TotalPushDistance += stationaryPush;
                        
                        // Mark unit as pushed if accumulated displacement is significant
                        if (ui.TotalPushDistance > GameConfig.PushDistanceThreshold && !ui.WasPushed)
                        {
                            ui.WasPushed = true;
                        }
                    }
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
