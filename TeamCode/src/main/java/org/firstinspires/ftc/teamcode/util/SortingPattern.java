package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;
import java.util.List;

/**
 * Slot ordering driven by AprilTag IDs read at the start of a match.
 */
public enum SortingPattern {
    GREEN_PURPLE_PURPLE(21, Arrays.asList(ColorTarget.GREEN, ColorTarget.PURPLE, ColorTarget.PURPLE)),
    PURPLE_GREEN_PURPLE(22, Arrays.asList(ColorTarget.PURPLE, ColorTarget.GREEN, ColorTarget.PURPLE)),
    PURPLE_PURPLE_GREEN(23, Arrays.asList(ColorTarget.PURPLE, ColorTarget.PURPLE, ColorTarget.GREEN));

    public final int sourceTagId;
    public final List<ColorTarget> slotTargets;

    SortingPattern(int sourceTagId, List<ColorTarget> slotTargets) {
        this.sourceTagId = sourceTagId;
        this.slotTargets = slotTargets;
    }

    public static SortingPattern fromTagId(int tagId) {
        for (SortingPattern p : values()) {
            if (p.sourceTagId == tagId) {
                return p;
            }
        }
        return PURPLE_PURPLE_GREEN;
    }

    public enum ColorTarget {
        GREEN,
        PURPLE
    }
}
