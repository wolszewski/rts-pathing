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
}
