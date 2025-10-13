// .NET 8 top-level program
// NuGet: Raylib-cs (namespace Raylib_cs)

using System.Numerics;
// ----------------------------
// Types & helpers
// ----------------------------
public struct Unit
{
    public Vector2 Pos;
    public Vector2 Vel;
    public float Radius;
    public bool Selected;
    public bool HasTarget;
    public Vector2 Target;
    public float Facing; // Angle in radians that the unit is currently facing
    
    // Stuck detection
    public float StuckTimer;          // How long unit has been stuck
    public float LastDistToTarget;    // Distance to target in previous frame
    
    // Group movement
    public int GroupId;               // Units with same GroupId belong to the same formation
    
    // Push recovery
    public Vector2 RestPosition;      // Position to return to when pushed (for stationary units)
    public bool WasPushed;            // Flag indicating unit was pushed and needs to return
    public float TotalPushDistance;   // Accumulates total distance pushed to trigger recovery
    public float PushRecoveryTimer;   // How long unit has been trying to return to rest (prevents infinite circling)
    
    // Pathfinding
    public List<Vector2>? Path;       // Current path waypoints (null if no path)
    public int CurrentWaypointIndex;  // Index of next waypoint to reach
    public bool IsGroupLeader;        // True if this unit calculates path for the group
}
