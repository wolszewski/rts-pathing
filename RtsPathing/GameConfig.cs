public static class GameConfig
{
    public const int ScreenW = 1280;
    public const int ScreenH = 720;
    public const int TargetFps = 240;           // render cap (high so vsync/off works smoothly)
    public const float TickDt = 0.01f;         // 5ms fixed tick (200 Hz)

    // Movement & rotation config
    public const float MaxSpeed = 120f;         // units per second
    public const float RotationSpeed = 15f;     // radians per second
    public const float SharpTurnAngleDeg = 15f; // degrees tolerance from 180° - unit stops to rotate if turn > (180 - this value)
    public const float ArriveDist = 4f;         // distance at which unit considers target reached
    
    // Stuck detection config
    public const float StuckTimeThreshold = 1.0f;   // seconds - how long before declaring unit stuck
    public const float StuckProgressThreshold = 1f; // units/sec - minimum progress speed to not be considered stuck
    
    // Collision avoidance config
    public const float AvoidanceRange = 15f;        // distance at which units start avoiding each other
    public const float AvoidanceStrength = 0.3f;    // strength of avoidance force (reduced from 100f - should be on same scale as desired direction)
    
    // Obstacle avoidance config
    public const float ObstacleDetectionRange = 40f;  // how far ahead to look for obstacles
    public const float ObstacleAvoidanceStrength = 1.5f; // strength of obstacle avoidance (stronger than unit avoidance)
    public const float ObstacleBounceReduction = 0.7f;   // velocity reduction when bouncing off obstacles (0.7 = 70% kept)
    
    // Pathfinding config
    public const int GridCellSize = 50;              // size of each grid cell in world units
    public const int GridWidth = 200;                // grid width in cells (200 * 50 = 10,000 world units)
    public const int GridHeight = 200;               // grid height in cells (200 * 50 = 10,000 world units)
    public const float PathfindingUpdateInterval = 0.5f; // seconds between path recalculations for groups
    
    // Formation config
    public const float FormationSpacing = 20f;      // space between units in formation
    public const bool UseCircularFormation = true; // true = circular, false = grid formation
    
    // Push recovery config
    public const float PushDistanceThreshold = 0.5f;  // minimum distance pushed to trigger recovery (lowered from 3f)
    public const float ReturnToRestSpeed = 80f;       // speed when returning to rest position (slower than MaxSpeed)
    
    // Collision push strength
    public const float MovingUnitPushRatio = 1f;   // How much moving unit pushes (0.75 = 75% to stationary, 25% pushback)
                                                       // Higher = moving unit pushes harder (closer to 1.0)
                                                       // Lower = more balanced (closer to 0.5)
    
    // World bounds (derived from grid)
    public static float WorldMinX => -GridWidth * GridCellSize * 0.5f;  // -5000
    public static float WorldMaxX => GridWidth * GridCellSize * 0.5f;   // 5000
    public static float WorldMinY => -GridHeight * GridCellSize * 0.5f; // -5000
    public static float WorldMaxY => GridHeight * GridCellSize * 0.5f;  // 5000
}
