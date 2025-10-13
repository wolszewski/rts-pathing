using System.Numerics;

public static class FormationCalculator
{
    private static int _nextGroupId = 1;
    
    /// <summary>
    /// Calculates formation positions for a group of units around a center point.
    /// Uses a square/grid formation with units arranged in rows and columns.
    /// </summary>
    public static Vector2[] CalculateGridFormation(Vector2 centerPos, int unitCount, float spacing = 20f)
    {
        if (unitCount == 0) return Array.Empty<Vector2>();
        if (unitCount == 1) return new[] { centerPos };

        var positions = new Vector2[unitCount];
        
        // Calculate grid dimensions - aim for roughly square formation
        int cols = (int)MathF.Ceiling(MathF.Sqrt(unitCount));
        int rows = (int)MathF.Ceiling((float)unitCount / cols);
        
        // Calculate offsets to center the formation
        float offsetX = (cols - 1) * spacing * 0.5f;
        float offsetY = (rows - 1) * spacing * 0.5f;
        
        int idx = 0;
        for (int row = 0; row < rows && idx < unitCount; row++)
        {
            for (int col = 0; col < cols && idx < unitCount; col++)
            {
                float x = col * spacing - offsetX;
                float y = row * spacing - offsetY;
                positions[idx] = centerPos + new Vector2(x, y);
                idx++;
            }
        }
        
        return positions;
    }
    
    /// <summary>
    /// Calculates formation positions in a circular/rounded pattern.
    /// Places units in concentric circles around the center point.
    /// </summary>
    public static Vector2[] CalculateCircularFormation(Vector2 centerPos, int unitCount, float spacing = 20f)
    {
        if (unitCount == 0) return Array.Empty<Vector2>();
        if (unitCount == 1) return new[] { centerPos };

        var positions = new Vector2[unitCount];
        positions[0] = centerPos; // First unit at center
        
        if (unitCount == 1) return positions;
        
        int idx = 1;
        int ring = 1;
        
        while (idx < unitCount)
        {
            float radius = ring * spacing;
            int unitsInRing = ring * 6; // Hexagonal packing: 6, 12, 18, 24...
            
            // Don't place more units than remaining
            int actualUnitsInRing = Math.Min(unitsInRing, unitCount - idx);
            
            for (int i = 0; i < actualUnitsInRing; i++)
            {
                float angle = (i / (float)actualUnitsInRing) * MathF.PI * 2f;
                float x = MathF.Cos(angle) * radius;
                float y = MathF.Sin(angle) * radius;
                positions[idx] = centerPos + new Vector2(x, y);
                idx++;
            }
            
            ring++;
        }
        
        return positions;
    }
    
    /// <summary>
    /// Assigns formation positions to units, trying to minimize total movement.
    /// Uses a simple greedy approach: each unit gets the closest available formation position.
    /// Also assigns all units to the same group ID for coordinated movement.
    /// </summary>
    public static void AssignFormationPositions(Unit[] units, int[] selectedIndices, Vector2[] formationPositions)
    {
        if (selectedIndices.Length != formationPositions.Length)
            throw new ArgumentException("Number of selected units must match formation positions");
        
        // Assign a new group ID for this formation
        int groupId = _nextGroupId++;
        
        var available = new List<Vector2>(formationPositions);
        
        // Assign each unit to the closest available position
        foreach (int unitIdx in selectedIndices)
        {
            if (available.Count == 0) break;
            
            var unit = units[unitIdx];
            int closestIdx = 0;
            float closestDist = Vector2.DistanceSquared(unit.Pos, available[0]);
            
            // Find closest available position
            for (int i = 1; i < available.Count; i++)
            {
                float dist = Vector2.DistanceSquared(unit.Pos, available[i]);
                if (dist < closestDist)
                {
                    closestDist = dist;
                    closestIdx = i;
                }
            }
            
            // Assign position, group ID, and remove from available
            var u = units[unitIdx];
            u.Target = available[closestIdx];
            u.HasTarget = true;
            u.StuckTimer = 0f;
            u.LastDistToTarget = Vector2.Distance(u.Pos, available[closestIdx]);
            u.GroupId = groupId;
            units[unitIdx] = u;
            
            available.RemoveAt(closestIdx);
        }
    }
}
