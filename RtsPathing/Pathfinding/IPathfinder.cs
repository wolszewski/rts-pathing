using System.Numerics;

namespace RtsPathing.Pathfinding;

/// <summary>
/// Interface for pathfinding implementations.
/// Allows easy swapping between A*, HPA*, NavMesh, Flow Fields, etc.
/// </summary>
public interface IPathfinder
{
    /// <summary>
    /// Find a path from start to goal position.
    /// Returns null if no path exists.
    /// </summary>
    List<Vector2>? FindPath(Vector2 start, Vector2 goal);
    
    /// <summary>
    /// Update the pathfinding grid with obstacle changes.
    /// Called when obstacles are added/removed.
    /// </summary>
    void UpdateObstacles(IEnumerable<(Vector2 A, Vector2 B)> obstacles);
}
