public static class GameConfig
{
    public const int ScreenW = 1280;
    public const int ScreenH = 720;
    public const int TargetFps = 240;           // render cap (high so vsync/off works smoothly)
    public const float TickDt = 0.005f;         // 5ms fixed tick (200 Hz)

    // Movement & rotation config
    public const float MaxSpeed = 120f;         // units per second
    public const float RotationSpeed = 15f;     // radians per second
    public const float SharpTurnAngleDeg = 15f; // degrees tolerance from 180° - unit stops to rotate if turn > (180 - this value)
    public const float ArriveDist = 6f;         // distance at which unit considers target reached
    
    // Stuck detection config
    public const float StuckTimeThreshold = 1.0f;   // seconds - how long before declaring unit stuck
    public const float StuckProgressThreshold = 1f; // units/sec - minimum progress speed to not be considered stuck
    
    // Collision avoidance config
    public const float AvoidanceRange = 25f;        // distance at which units start avoiding each other
    public const float AvoidanceStrength = 100f;    // strength of avoidance force
}
