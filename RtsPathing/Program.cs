// .NET 8 top-level program
// NuGet: Raylib-cs (namespace Raylib_cs)

using System.Numerics;
using Raylib_cs;
using RtsPathing.Pathfinding;

// Initialize pathfinding grid
var pathGrid = new PathfindingGrid(GameConfig.GridWidth, GameConfig.GridHeight, GameConfig.GridCellSize);

// Build map with obstacles and unit positions
var (obstacles, unitPositions) = MapBuilder.BuildDefaultMap(unitCount: 100, seed: 42);

// Initialize pathfinder
IPathfinder pathfinder = new AStarPathfinder(pathGrid);
pathfinder.UpdateObstacles(obstacles);

// Camera params
float zoom = 0.15f; // Start zoomed out to see the larger map
var cam = new Camera2D
{
    Target = Vector2.Zero,
    Offset = new Vector2(GameConfig.ScreenW / 2f, GameConfig.ScreenH / 2f),
    Rotation = 0f,
    Zoom = zoom
};

// Units
var rng = new Random(42);
var units = new Unit[unitPositions.Length];
for (int i = 0; i < unitPositions.Length; i++)
{
    var p = unitPositions[i];
    units[i] = new Unit
    {
        Pos = p,
        Vel = Vector2.Zero,
        Radius = 4f + (float)rng.NextDouble() * 10f,
        Selected = false,
        HasTarget = false,
        Target = p,
        Facing = 0f,
        StuckTimer = 0f,
        LastDistToTarget = 0f,
        GroupId = 0,
        RestPosition = p,
        WasPushed = false,
        TotalPushDistance = 0f,
        PushRecoveryTimer = 0f,
        Path = null,
        CurrentWaypointIndex = 0,
        IsGroupLeader = false
    };
}

// Init window
Raylib.SetConfigFlags(ConfigFlags.VSyncHint);
Raylib.InitWindow(GameConfig.ScreenW, GameConfig.ScreenH, "2D RTS Pathing with A* (200x200 grid)");
Raylib.SetTargetFPS(GameConfig.TargetFps);

// Fixed timestep accumulator
double accumulator = 0.0;
double currentTime = Raylib.GetTime();
long tickCounter = 0;

// Input settings
float camPanSpeed = 2000f; // Faster for larger map
float zoomSpeed = 1.0f;

// Selection state
bool isSelecting = false;
Vector2 selectStartScreen = Vector2.Zero;
Vector2 selectEndScreen = Vector2.Zero;
const float ClickDragThreshold = 5f;

while (!Raylib.WindowShouldClose())
{
    // Time step management
    double newTime = Raylib.GetTime();
    double frameTime = newTime - currentTime;
    currentTime = newTime;
    accumulator += frameTime;

    // Handle input (camera)
    var worldPan = camPanSpeed * (float)frameTime / cam.Zoom;

    if (Raylib.IsKeyDown(KeyboardKey.W)) cam.Target.Y -= worldPan;
    if (Raylib.IsKeyDown(KeyboardKey.S)) cam.Target.Y += worldPan;
    if (Raylib.IsKeyDown(KeyboardKey.A)) cam.Target.X -= worldPan;
    if (Raylib.IsKeyDown(KeyboardKey.D)) cam.Target.X += worldPan;

    if (Raylib.IsKeyDown(KeyboardKey.Q)) zoom *= (1f + zoomSpeed * (float)frameTime);
    if (Raylib.IsKeyDown(KeyboardKey.E)) zoom *= (1f - zoomSpeed * (float)frameTime);
    zoom = Math.Clamp(zoom, 0.05f, 2f); // Wider zoom range for big map
    cam.Zoom = zoom;

    if (Raylib.IsKeyPressed(KeyboardKey.R))
    {
        cam.Target = Vector2.Zero;
        cam.Zoom = zoom = 0.15f;
    }

    // Mouse wheel zoom
    float mw = Raylib.GetMouseWheelMove();
    if (Math.Abs(mw) > 0.0001f)
    {
        var mouseScreen = Raylib.GetMousePosition();
        var before = Raylib.GetScreenToWorld2D(mouseScreen, cam);
        zoom *= (mw > 0 ? 1.1f : 0.9f);
        zoom = Math.Clamp(zoom, 0.05f, 2f);
        cam.Zoom = zoom;
        var after = Raylib.GetScreenToWorld2D(mouseScreen, cam);
        cam.Target += before - after;
    }

    // Mouse selection input
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
            // Click selection
            var clickWorld = Raylib.GetScreenToWorld2D(selectEndScreen, cam);
            int bestIdx = -1;
            float bestDistSq = float.MaxValue;
            for (int i = 0; i < units.Length; i++)
            {
                var u = units[i];
                float d2 = Vector2.DistanceSquared(u.Pos, clickWorld);
                if (d2 <= u.Radius * u.Radius && d2 < bestDistSq)
                {
                    bestDistSq = d2;
                    bestIdx = i;
                }
            }

            if (bestIdx >= 0)
            {
                for (int i = 0; i < units.Length; i++)
                {
                    var u = units[i];
                    u.Selected = (i == bestIdx);
                    units[i] = u;
                }
            }
        }
        else
        {
            // Box selection
            Vector2 w0 = Raylib.GetScreenToWorld2D(selectStartScreen, cam);
            Vector2 w1 = Raylib.GetScreenToWorld2D(selectEndScreen, cam);
            float minX = MathF.Min(w0.X, w1.X);
            float maxX = MathF.Max(w0.X, w1.X);
            float minY = MathF.Min(w0.Y, w1.Y);
            float maxY = MathF.Max(w0.Y, w1.Y);

            for (int i = 0; i < units.Length; i++)
            {
                var u = units[i];
                u.Selected = u.Pos.X >= minX && u.Pos.X <= maxX && u.Pos.Y >= minY && u.Pos.Y <= maxY;
                units[i] = u;
            }
        }

        isSelecting = false;
    }

    // Issue move command with pathfinding
    if (Raylib.IsMouseButtonPressed(MouseButton.Right))
    {
        var clickWorld = Raylib.GetScreenToWorld2D(Raylib.GetMousePosition(), cam);
        
        var selectedIndices = new List<int>();
        for (int i = 0; i < units.Length; i++)
        {
            if (units[i].Selected)
                selectedIndices.Add(i);
        }
        
        if (selectedIndices.Count > 0)
        {
            // Calculate formation positions
            Vector2[] formationPositions;
            if (GameConfig.UseCircularFormation)
            {
                formationPositions = FormationCalculator.CalculateCircularFormation(
                    clickWorld, selectedIndices.Count, GameConfig.FormationSpacing,
                    units, selectedIndices.ToArray());
            }
            else
            {
                formationPositions = FormationCalculator.CalculateGridFormation(
                    clickWorld, selectedIndices.Count, GameConfig.FormationSpacing,
                    units, selectedIndices.ToArray());
            }
            
            // Assign formation and calculate path for group leader
            FormationCalculator.AssignFormationPositions(units, selectedIndices.ToArray(), formationPositions);
            
            // Find group leader (closest to formation center)
            int leaderIdx = selectedIndices[0];
            float minDist = Vector2.DistanceSquared(units[leaderIdx].Pos, clickWorld);
            for (int i = 1; i < selectedIndices.Count; i++)
            {
                float d = Vector2.DistanceSquared(units[selectedIndices[i]].Pos, clickWorld);
                if (d < minDist)
                {
                    minDist = d;
                    leaderIdx = selectedIndices[i];
                }
            }
            
            // Calculate paths for all units in the group
            // Leader gets a direct path, followers use leader's path as guidance
            var leaderPath = pathfinder.FindPath(units[leaderIdx].Pos, units[leaderIdx].Target);
            
            // If leader has no path, give all units direct paths
            if (leaderPath == null || leaderPath.Count == 0)
            {
                // No pathfinding available, use direct movement
                for (int i = 0; i < selectedIndices.Count; i++)
                {
                    int unitIdx = selectedIndices[i];
                    var u = units[unitIdx];
                    u.IsGroupLeader = (unitIdx == leaderIdx);
                    u.Path = null; // Will use direct movement with local avoidance
                    u.CurrentWaypointIndex = 0;
                    units[unitIdx] = u;
                }
            }
            else
            {
                // Check if group is split across obstacles by testing if all units
                // can reach the leader's first waypoint without major detours
                bool groupIsSplit = false;
                if (leaderPath.Count > 1)
                {
                    Vector2 leaderFirstWaypoint = leaderPath[1]; // Second point (after start)
                    int unitsOnOtherSide = 0;
                    
                    foreach (int unitIdx in selectedIndices)
                    {
                        if (unitIdx == leaderIdx) continue;
                        
                        // Simple heuristic: if direct distance to waypoint is much larger
                        // than leader's path distance, unit is probably on other side of obstacle
                        float directDist = Vector2.Distance(units[unitIdx].Pos, leaderFirstWaypoint);
                        float leaderDist = Vector2.Distance(units[leaderIdx].Pos, leaderFirstWaypoint);
                        
                        if (directDist > leaderDist * 2.0f) // 2x threshold indicates split
                        {
                            unitsOnOtherSide++;
                        }
                    }
                    
                    // If more than 30% of units are on the other side, group is split
                    groupIsSplit = unitsOnOtherSide > selectedIndices.Count * 0.3f;
                }
                
                // Assign paths based on whether group is split
                for (int i = 0; i < selectedIndices.Count; i++)
                {
                    int unitIdx = selectedIndices[i];
                    var u = units[unitIdx];
                    u.IsGroupLeader = (unitIdx == leaderIdx);
                    
                    if (unitIdx == leaderIdx)
                    {
                        // Leader uses direct path to formation center
                        u.Path = leaderPath;
                    }
                    else if (groupIsSplit)
                    {
                        // Group is split - calculate individual path for each follower
                        var individualPath = pathfinder.FindPath(u.Pos, u.Target);
                        
                        // If individual path fails, try adapted path as fallback
                        if (individualPath == null || individualPath.Count <= 2)
                        {
                            u.Path = AdaptLeaderPathForFollower(u.Pos, u.Target, leaderPath);
                        }
                        else
                        {
                            u.Path = individualPath;
                        }
                    }
                    else
                    {
                        // Group is together - followers adapt leader's path
                        u.Path = AdaptLeaderPathForFollower(units[unitIdx].Pos, units[unitIdx].Target, leaderPath);
                    }
                    
                    u.CurrentWaypointIndex = 0;
                    units[unitIdx] = u;
                }
            }
        }
    }

    // Fixed update loop
    while (accumulator >= GameConfig.TickDt)
    {
        FixedUpdate((float)GameConfig.TickDt, units, obstacles.ToArray(), pathfinder);
        accumulator -= GameConfig.TickDt;
        tickCounter++;
    }

    // Render
    Raylib.BeginDrawing();
    Raylib.ClearBackground(Color.DarkGray);

    Raylib.BeginMode2D(cam);

    Renderer.DrawGrid(100, GameConfig.GridCellSize); // Show 100×100 cells (half of 200×200)
    Renderer.DrawObstacles(obstacles.ToArray());
    DrawPaths(units); // Draw paths before units
    Renderer.DrawUnits(units);
    
    Raylib.EndMode2D();

    if (isSelecting)
    {
        Renderer.DrawSelectionBox(selectStartScreen, selectEndScreen);
    }

    Renderer.DrawOverlay(zoom, units, tickCounter);

    Raylib.EndDrawing();
}

Raylib.CloseWindow();

/// <summary>
/// Adapts the leader's path for a follower unit by offsetting waypoints
/// while maintaining the same general route (keeping group coherence).
/// </summary>
static List<Vector2>? AdaptLeaderPathForFollower(Vector2 followerStart, Vector2 followerGoal, List<Vector2>? leaderPath)
{
    // If leader has no path or very short path, follower gets direct path
    if (leaderPath == null || leaderPath.Count <= 2)
        return new List<Vector2> { followerStart, followerGoal };
    
    // Calculate offset from leader's start to follower's start
    Vector2 startOffset = followerStart - leaderPath[0];
    
    // Calculate offset from leader's goal to follower's goal
    Vector2 goalOffset = followerGoal - leaderPath[^1];
    
    // Create adapted path by interpolating the offset along the route
    var adaptedPath = new List<Vector2>();
    
    for (int i = 0; i < leaderPath.Count; i++)
    {
        // Interpolation factor (0 at start, 1 at goal)
        float t = i / (float)(leaderPath.Count - 1);
        
        // Blend between start offset and goal offset
        Vector2 offset = Vector2.Lerp(startOffset, goalOffset, t);
        
        // Apply offset to leader's waypoint
        Vector2 adaptedWaypoint = leaderPath[i] + offset;
        adaptedPath.Add(adaptedWaypoint);
    }
    
    // Ensure exact start and goal positions
    if (adaptedPath.Count > 0)
    {
        adaptedPath[0] = followerStart;
        adaptedPath[^1] = followerGoal;
    }
    
    return adaptedPath;
}


static void DrawPaths(Unit[] units)
{
    foreach (ref var u in units.AsSpan())
    {
        if (u.Path != null && u.Path.Count > 1)
        {
            // Draw path waypoints
            for (int i = u.CurrentWaypointIndex; i < u.Path.Count - 1; i++)
            {
                Raylib.DrawLineEx(u.Path[i], u.Path[i + 1], 2f, Color.Green);
                Raylib.DrawCircleV(u.Path[i], 3f, Color.Lime);
            }
            if (u.Path.Count > 0)
            {
                Raylib.DrawCircleV(u.Path[^1], 4f, Color.Green);
            }
        }
    }
}

static void FixedUpdate(float dt, Unit[] units, (Vector2 A, Vector2 B)[] obstacles, IPathfinder pathfinder)
{
    float sharpTurnThreshold = (180f - GameConfig.SharpTurnAngleDeg) * MathF.PI / 180f;

    // 1) Move towards targets with pathfinding and collision avoidance
    for (int i = 0; i < units.Length; i++)
    {
        var u = units[i];

        // Check if unit was pushed and needs to return to rest position
        if (u.WasPushed && !u.HasTarget)
        {
            Vector2 toRest = u.RestPosition - u.Pos;
            float distToRest = toRest.Length();
            
            u.PushRecoveryTimer += dt;
            
            if (u.PushRecoveryTimer > 3.0f)
            {
                u.WasPushed = false;
                u.Vel = Vector2.Zero;
                u.TotalPushDistance = 0f;
                u.PushRecoveryTimer = 0f;
                u.RestPosition = u.Pos;
            }
            else
            {
                float arriveThreshold = GameConfig.ArriveDist;
                if (distToRest < 15f)
                {
                    int nearbyCount = 0;
                    for (int k = 0; k < units.Length; k++)
                    {
                        if (k == i) continue;
                        float nearDist = Vector2.Distance(u.Pos, units[k].Pos);
                        if (nearDist < u.Radius + units[k].Radius + 5f)
                            nearbyCount++;
                    }
                    if (nearbyCount >= 2)
                        arriveThreshold = GameConfig.ArriveDist * 2.5f;
                }
                
                if (distToRest <= arriveThreshold)
                {
                    u.WasPushed = false;
                    u.Vel = Vector2.Zero;
                    u.TotalPushDistance = 0f;
                    u.PushRecoveryTimer = 0f;
                }
                else
                {
                    var dir = toRest / distToRest;
                    float targetAngle = MathF.Atan2(dir.Y, dir.X);
                    
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
                    
                    float speedScale = distToRest > 20f ? 1f : Math.Max(0.4f, distToRest / 20f);
                    Vector2 currentDir = new Vector2(MathF.Cos(u.Facing), MathF.Sin(u.Facing));
                    u.Vel = currentDir * GameConfig.ReturnToRestSpeed * speedScale;
                }
            }
        }
        else if (u.HasTarget)
        {
            // Determine actual target (next waypoint if following path, or final target)
            Vector2 currentTarget = u.Target;
            if (u.Path != null && u.CurrentWaypointIndex < u.Path.Count)
            {
                currentTarget = u.Path[u.CurrentWaypointIndex];
            }
            
            Vector2 to = currentTarget - u.Pos;
            float dist = to.Length();
            
            // Check if reached waypoint
            float waypointThreshold = u.Path != null ? GameConfig.ArriveDist * 3f : GameConfig.ArriveDist;
            
            if (dist <= waypointThreshold)
            {
                // Reached waypoint, advance to next
                if (u.Path != null && u.CurrentWaypointIndex < u.Path.Count - 1)
                {
                    u.CurrentWaypointIndex++;
                }
                else
                {
                    // Reached final target
                    float arriveThreshold = GameConfig.ArriveDist;
                    if (dist < 15f)
                    {
                        int nearbyCount = 0;
                        for (int k = 0; k < units.Length; k++)
                        {
                            if (k == i) continue;
                            float nearDist = Vector2.Distance(u.Pos, units[k].Pos);
                            if (nearDist < u.Radius + units[k].Radius + 5f)
                                nearbyCount++;
                        }
                        if (nearbyCount >= 2)
                            arriveThreshold = GameConfig.ArriveDist * 2.5f;
                    }
                    
                    if (dist <= arriveThreshold)
                    {
                        u.HasTarget = false;
                        u.Vel = Vector2.Zero;
                        u.StuckTimer = 0f;
                        u.RestPosition = u.Pos;
                        u.WasPushed = false;
                        u.Path = null;
                        u.CurrentWaypointIndex = 0;
                    }
                }
            }
            
            if (u.HasTarget)
            {
                // Recalculate target after potential waypoint advance
                if (u.Path != null && u.CurrentWaypointIndex < u.Path.Count)
                {
                    currentTarget = u.Path[u.CurrentWaypointIndex];
                }
                
                to = currentTarget - u.Pos;
                dist = to.Length();
                
                // Stuck detection
                float progressRate = (u.LastDistToTarget - dist) / dt;
                
                if (progressRate < GameConfig.StuckProgressThreshold)
                {
                    u.StuckTimer += dt;
                    
                    if (u.StuckTimer >= GameConfig.StuckTimeThreshold)
                    {
                        // Try recalculating path if we're the leader
                        if (u.IsGroupLeader && u.Path != null)
                        {
                            u.Path = pathfinder.FindPath(u.Pos, u.Target);
                            u.CurrentWaypointIndex = 0;
                            u.StuckTimer = 0f;
                        }
                        else
                        {
                            // Give up
                            u.HasTarget = false;
                            u.Vel = Vector2.Zero;
                            u.StuckTimer = 0f;
                            u.RestPosition = u.Pos;
                            u.Path = null;
                        }
                    }
                }
                else
                {
                    u.StuckTimer = 0f;
                }
                
                u.LastDistToTarget = dist;
                
                if (u.HasTarget && dist > 0.001f)
                {
                    var desiredDir = to / dist;
                    
                    float arrivalFactor = 1f - Math.Clamp((dist - GameConfig.ArriveDist) / 50f, 0f, 1f);
                    
                    Vector2 avoidanceForce = Vector2.Zero;
                    
                    if (dist > GameConfig.ArriveDist * 2f)
                    {
                        for (int j = 0; j < units.Length; j++)
                        {
                            if (i == j) continue;
                            
                            var other = units[j];
                            Vector2 toOther = other.Pos - u.Pos;
                            float distToOther = toOther.Length();
                            float avoidRadius = u.Radius + other.Radius + GameConfig.AvoidanceRange;
                            
                            if (distToOther < avoidRadius && distToOther > 0.001f)
                            {
                                Vector2 avoidDir = -toOther / distToOther;
                                float strength = (1f - (distToOther / avoidRadius));
                                
                                if (other.Vel.LengthSquared() < 10f * 10f)
                                {
                                    float avoidanceScale = (1f - arrivalFactor * 0.8f);
                                    avoidanceForce += avoidDir * strength * GameConfig.AvoidanceStrength * avoidanceScale;
                                }
                            }
                        }
                    }
                    
                    Vector2 obstacleAvoidance = CalculateObstacleAvoidance(u, obstacles, GameConfig.ObstacleDetectionRange);
                    
                    Vector2 combinedDir = desiredDir + avoidanceForce * (1f - arrivalFactor * 0.9f) + obstacleAvoidance;
                    
                    if (combinedDir.LengthSquared() > 0.001f)
                    {
                        combinedDir = Vector2.Normalize(combinedDir);
                    }
                    else
                    {
                        combinedDir = desiredDir;
                    }
                    
                    float targetAngle = MathF.Atan2(combinedDir.Y, combinedDir.X);
                    float initialAngleDiff = NormalizeAngle(targetAngle - u.Facing);
                    bool isSharpTurn = MathF.Abs(initialAngleDiff) > sharpTurnThreshold;
                    
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
                    
                    if (isSharpTurn)
                    {
                        u.Vel = Vector2.Zero;
                    }
                    else
                    {
                        float speedScale = dist > 30f ? 1f : Math.Max(0.3f, dist / 30f);
                        Vector2 currentDir = new Vector2(MathF.Cos(u.Facing), MathF.Sin(u.Facing));
                        u.Vel = currentDir * GameConfig.MaxSpeed * speedScale;
                    }
                }
            }
        }
        else
        {
            u.Vel = Vector2.Zero;
            u.StuckTimer = 0f;
        }

        u.Pos += u.Vel * dt;

        HandleObstacleCollision(ref u, obstacles, dt);

        // World bounds using config values
        float B = GameConfig.WorldMaxX;
        if (u.Pos.X < -B || u.Pos.X > B) { u.Vel.X = 0; u.Pos.X = Math.Clamp(u.Pos.X, -B, B); u.HasTarget = false; }
        if (u.Pos.Y < -B || u.Pos.Y > B) { u.Vel.Y = 0; u.Pos.Y = Math.Clamp(u.Pos.Y, -B, B); u.HasTarget = false; }

        units[i] = u;
    }

    // 2) Unit-unit collision resolution
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
                
                bool iMoving = (ui.HasTarget || ui.WasPushed) && ui.Vel.LengthSquared() > 1f;
                bool jMoving = (uj.HasTarget || uj.WasPushed) && uj.Vel.LengthSquared() > 1f;
                bool sameGroup = ui.GroupId > 0 && ui.GroupId == uj.GroupId;
                
                if (!iMoving && !jMoving)
                {
                    ui.Pos -= n * (penetration * 0.5f);
                    uj.Pos += n * (penetration * 0.5f);
                }
                else if (iMoving && !jMoving)
                {
                    if (sameGroup)
                    {
                        float movingPush = penetration * (1f - GameConfig.MovingUnitPushRatio);
                        float stationaryPush = penetration * GameConfig.MovingUnitPushRatio;
                        
                        ui.Pos -= n * movingPush;
                        uj.Pos += n * stationaryPush;
                        
                        if (!uj.HasTarget && Vector2.Distance(uj.Pos, uj.Target) > GameConfig.ArriveDist)
                        {
                            uj.HasTarget = true;
                            uj.StuckTimer = 0f;
                            uj.LastDistToTarget = Vector2.Distance(uj.Pos, uj.Target);
                            uj.WasPushed = false;
                            uj.TotalPushDistance = 0f;
                        }
                    }
                    else
                    {
                        float movingPush = penetration * (1f - GameConfig.MovingUnitPushRatio);
                        float stationaryPush = penetration * GameConfig.MovingUnitPushRatio;
                        
                        ui.Pos -= n * movingPush;
                        uj.Pos += n * stationaryPush;
                        uj.TotalPushDistance += stationaryPush;
                        
                        if (uj.TotalPushDistance > GameConfig.PushDistanceThreshold && !uj.WasPushed)
                        {
                            uj.WasPushed = true;
                        }
                    }
                }
                else if (!iMoving && jMoving)
                {
                    if (sameGroup)
                    {
                        float movingPush = penetration * (1f - GameConfig.MovingUnitPushRatio);
                        float stationaryPush = penetration * GameConfig.MovingUnitPushRatio;
                        
                        ui.Pos -= n * stationaryPush;
                        uj.Pos += n * movingPush;
                        
                        if (!ui.HasTarget && Vector2.Distance(ui.Pos, ui.Target) > GameConfig.ArriveDist)
                        {
                            ui.HasTarget = true;
                            ui.StuckTimer = 0f;
                            ui.LastDistToTarget = Vector2.Distance(ui.Pos, ui.Target);
                            ui.WasPushed = false;
                            ui.TotalPushDistance = 0f;
                        }
                    }
                    else
                    {
                        float movingPush = penetration * (1f - GameConfig.MovingUnitPushRatio);
                        float stationaryPush = penetration * GameConfig.MovingUnitPushRatio;
                        
                        ui.Pos -= n * stationaryPush;
                        uj.Pos += n * movingPush;
                        ui.TotalPushDistance += stationaryPush;
                        
                        if (ui.TotalPushDistance > GameConfig.PushDistanceThreshold && !ui.WasPushed)
                        {
                            ui.WasPushed = true;
                        }
                    }
                }
                else
                {
                    ui.Pos -= n * (penetration * 0.5f);
                    uj.Pos += n * (penetration * 0.5f);
                }
                
                units[i] = ui;
                units[j] = uj;
            }
        }
    }
}

static float NormalizeAngle(float angle)
{
    while (angle > MathF.PI) angle -= MathF.PI * 2;
    while (angle < -MathF.PI) angle += MathF.PI * 2;
    return angle;
}

static Vector2 ClosestPointOnSegment(Vector2 point, Vector2 segA, Vector2 segB)
{
    Vector2 ab = segB - segA;
    float t = Vector2.Dot(point - segA, ab) / Vector2.Dot(ab, ab);
    t = Math.Clamp(t, 0f, 1f);
    return segA + ab * t;
}

static bool CircleSegmentIntersect(Vector2 circlePos, float radius, Vector2 segA, Vector2 segB, out Vector2 closestPoint, out float distance)
{
    closestPoint = ClosestPointOnSegment(circlePos, segA, segB);
    distance = Vector2.Distance(circlePos, closestPoint);
    return distance <= radius;
}

static Vector2 CalculateObstacleAvoidance(Unit unit, (Vector2 A, Vector2 B)[] obstacles, float detectionRange)
{
    Vector2 avoidanceForce = Vector2.Zero;
    
    if (unit.Vel.LengthSquared() < 0.1f)
        return avoidanceForce;
    
    Vector2 velocityDir = Vector2.Normalize(unit.Vel);
    
    foreach (var obstacle in obstacles)
    {
        Vector2 closestPoint = ClosestPointOnSegment(unit.Pos, obstacle.A, obstacle.B);
        float distToObstacle = Vector2.Distance(unit.Pos, closestPoint);
        
        if (distToObstacle > detectionRange + unit.Radius)
            continue;
        
        Vector2 awayFromObstacle = unit.Pos - closestPoint;
        if (awayFromObstacle.LengthSquared() < 0.001f)
        {
            Vector2 obstacleDir = Vector2.Normalize(obstacle.B - obstacle.A);
            awayFromObstacle = new Vector2(-obstacleDir.Y, obstacleDir.X);
        }
        else
        {
            awayFromObstacle = Vector2.Normalize(awayFromObstacle);
        }
        
        float approachDot = Vector2.Dot(velocityDir, -awayFromObstacle);
        if (approachDot <= 0)
            continue;
        
        float distanceFactor = 1f - Math.Clamp(distToObstacle / (detectionRange + unit.Radius), 0f, 1f);
        float approachFactor = approachDot;
        
        float strength = distanceFactor * distanceFactor * approachFactor * GameConfig.ObstacleAvoidanceStrength;
        avoidanceForce += awayFromObstacle * strength;
    }
    
    return avoidanceForce;
}

static void HandleObstacleCollision(ref Unit unit, (Vector2 A, Vector2 B)[] obstacles, float dt)
{
    foreach (var obstacle in obstacles)
    {
        if (CircleSegmentIntersect(unit.Pos, unit.Radius, obstacle.A, obstacle.B, out Vector2 closestPoint, out float distance))
        {
            float penetration = unit.Radius - distance;
            
            if (penetration > 0)
            {
                Vector2 pushDir = unit.Pos - closestPoint;
                if (pushDir.LengthSquared() < 0.001f)
                {
                    Vector2 obstacleDir = Vector2.Normalize(obstacle.B - obstacle.A);
                    pushDir = new Vector2(-obstacleDir.Y, obstacleDir.X);
                }
                else
                {
                    pushDir = Vector2.Normalize(pushDir);
                }
                
                unit.Pos += pushDir * penetration;
                
                if (unit.Vel.LengthSquared() > 0.001f)
                {
                    Vector2 obstacleDir = Vector2.Normalize(obstacle.B - obstacle.A);
                    Vector2 normal = new Vector2(-obstacleDir.Y, obstacleDir.X);
                    
                    if (Vector2.Dot(normal, pushDir) < 0)
                        normal = -normal;
                    
                    float slideComponent = Vector2.Dot(unit.Vel, obstacleDir);
                    Vector2 slideVel = obstacleDir * slideComponent;
                    
                    float bounceComponent = Vector2.Dot(unit.Vel, normal);
                    
                    if (bounceComponent < 0)
                    {
                        Vector2 bounceVel = -normal * bounceComponent * GameConfig.ObstacleBounceReduction;
                        unit.Vel = slideVel * 0.9f + bounceVel;
                    }
                }
            }
        }
    }
}
