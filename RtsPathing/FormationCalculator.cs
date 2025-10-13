using System.Numerics;

public static class FormationCalculator
{
    private static int _nextGroupId = 1;
    
    /// <summary>
    /// Calculates formation positions for a group of units around a center point.
    /// Uses a square/grid formation with units arranged in rows and columns.
    /// Accounts for unit sizes to prevent overlapping.
    /// </summary>
    public static Vector2[] CalculateGridFormation(Vector2 centerPos, int unitCount, float spacing = 20f, Unit[]? units = null, int[]? selectedIndices = null)
    {
        if (unitCount == 0) return Array.Empty<Vector2>();
        if (unitCount == 1) return new[] { centerPos };

        var positions = new Vector2[unitCount];
        
        // Calculate grid dimensions - aim for roughly square formation
        int cols = (int)MathF.Ceiling(MathF.Sqrt(unitCount));
        int rows = (int)MathF.Ceiling((float)unitCount / cols);
        
        // If units are provided, calculate size-aware spacing
        float[] rowHeights = new float[rows];
        float[] colWidths = new float[cols];
        
        if (units != null && selectedIndices != null)
        {
            // Calculate row heights and column widths based on unit sizes
            int idx = 0;
            for (int row = 0; row < rows && idx < unitCount; row++)
            {
                float maxRadiusInRow = 0f;
                for (int col = 0; col < cols && idx < unitCount; col++)
                {
                    float radius = units[selectedIndices[idx]].Radius;
                    maxRadiusInRow = MathF.Max(maxRadiusInRow, radius);
                    colWidths[col] = MathF.Max(colWidths[col], radius);
                    idx++;
                }
                rowHeights[row] = maxRadiusInRow;
            }
            
            // Add spacing between units
            for (int i = 0; i < rows; i++)
                rowHeights[i] = rowHeights[i] * 2f + spacing;
            for (int i = 0; i < cols; i++)
                colWidths[i] = colWidths[i] * 2f + spacing;
        }
        else
        {
            // Use uniform spacing if no unit data provided
            for (int i = 0; i < rows; i++)
                rowHeights[i] = spacing;
            for (int i = 0; i < cols; i++)
                colWidths[i] = spacing;
        }
        
        // Calculate total width and height
        float totalWidth = colWidths.Sum();
        float totalHeight = rowHeights.Sum();
        
        // Calculate starting position to center the formation
        float startX = -totalWidth * 0.5f;
        float startY = -totalHeight * 0.5f;
        
        // Place units
        int unitIdx = 0;
        float currentY = startY;
        for (int row = 0; row < rows && unitIdx < unitCount; row++)
        {
            float currentX = startX;
            for (int col = 0; col < cols && unitIdx < unitCount; col++)
            {
                float x = currentX + colWidths[col] * 0.5f;
                float y = currentY + rowHeights[row] * 0.5f;
                positions[unitIdx] = centerPos + new Vector2(x, y);
                currentX += colWidths[col];
                unitIdx++;
            }
            currentY += rowHeights[row];
        }
        
        return positions;
    }
    
    /// <summary>
    /// Calculates formation positions in a circular/rounded pattern.
    /// Places units in concentric circles around the center point.
    /// Accounts for unit sizes to prevent overlapping.
    /// </summary>
    public static Vector2[] CalculateCircularFormation(Vector2 centerPos, int unitCount, float spacing = 20f, Unit[]? units = null, int[]? selectedIndices = null)
    {
        if (unitCount == 0) return Array.Empty<Vector2>();
        if (unitCount == 1) return new[] { centerPos };

        var positions = new Vector2[unitCount];
        
        // Place first unit at center (typically the largest or first selected)
        positions[0] = centerPos;
        
        if (unitCount == 1) return positions;
        
        int idx = 1;
        int ring = 1;
        
        // Calculate base radius for first ring
        float baseRadius = spacing;
        if (units != null && selectedIndices != null && selectedIndices.Length > 0)
        {
            // Start ring outside the center unit
            float centerRadius = units[selectedIndices[0]].Radius;
            baseRadius = centerRadius + spacing;
        }
        
        while (idx < unitCount)
        {
            // Calculate maximum radius in this ring (for size-aware spacing)
            float maxRadiusInRing = 0f;
            if (units != null && selectedIndices != null)
            {
                int unitsToCheck = Math.Min(ring * 6, unitCount - idx);
                for (int i = 0; i < unitsToCheck && (idx + i) < selectedIndices.Length; i++)
                {
                    maxRadiusInRing = MathF.Max(maxRadiusInRing, units[selectedIndices[idx + i]].Radius);
                }
            }
            
            float radius = baseRadius + (ring - 1) * spacing;
            if (maxRadiusInRing > 0)
            {
                // Add space for the unit sizes
                radius += maxRadiusInRing;
            }
            
            int unitsInRing = ring * 6; // Hexagonal packing: 6, 12, 18, 24...
            
            // Don't place more units than remaining
            int actualUnitsInRing = Math.Min(unitsInRing, unitCount - idx);
            
            // Calculate angular spacing considering unit sizes
            float angularSpacing = (MathF.PI * 2f) / actualUnitsInRing;
            
            // If we have unit size info, adjust to prevent overlaps
            if (units != null && selectedIndices != null && idx < selectedIndices.Length)
            {
                // Calculate minimum angular spacing needed to prevent overlaps
                float avgRadius = maxRadiusInRing > 0 ? maxRadiusInRing : 5f;
                float arcLength = avgRadius * 2f + spacing * 0.5f;
                float minAngularSpacing = arcLength / radius;
                
                // Use the larger of the two to ensure spacing
                angularSpacing = MathF.Max(angularSpacing, minAngularSpacing);
                
                // Adjust actual units in ring if spacing requires it
                actualUnitsInRing = Math.Min(actualUnitsInRing, (int)(MathF.PI * 2f / angularSpacing));
            }
            
            for (int i = 0; i < actualUnitsInRing && idx < unitCount; i++)
            {
                float angle = i * angularSpacing;
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
    /// Now size-aware to prevent overlapping.
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
            u.WasPushed = false; // Clear any push state when given new target
            u.TotalPushDistance = 0f; // Reset accumulated push distance
            u.PushRecoveryTimer = 0f; // Reset recovery timer when given new command
            u.Path = null; // Clear any existing path (will be calculated by leader)
            u.CurrentWaypointIndex = 0;
            u.IsGroupLeader = false; // Leader will be assigned separately
            // RestPosition will be updated when unit reaches target
            units[unitIdx] = u;
            
            available.RemoveAt(closestIdx);
        }
    }
}
