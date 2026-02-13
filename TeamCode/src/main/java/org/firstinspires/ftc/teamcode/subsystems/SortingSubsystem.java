package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.SortingPattern;

/**
 * 3-slot sorter with 2 color sensors per slot and one shovel servo per slot.
 */
public class SortingSubsystem {
    private final Slot[] slots = new Slot[3];
    private SortingPattern currentPattern = SortingPattern.PURPLE_PURPLE_GREEN;

    public SortingSubsystem(
            ColorSensor s1a, ColorSensor s1b, Servo shovel1,
            ColorSensor s2a, ColorSensor s2b, Servo shovel2,
            ColorSensor s3a, ColorSensor s3b, Servo shovel3) {
        slots[0] = new Slot(s1a, s1b, shovel1);
        slots[1] = new Slot(s2a, s2b, shovel2);
        slots[2] = new Slot(s3a, s3b, shovel3);
    }

    public void setPattern(SortingPattern pattern) {
        this.currentPattern = pattern;
    }

    public SortingPattern getPattern() {
        return currentPattern;
    }

    /** Returns true when each slot matches desired color target. */
    public boolean isSorted() {
        for (int i = 0; i < slots.length; i++) {
            SortingPattern.ColorTarget observed = slots[i].classify();
            if (observed != currentPattern.slotTargets.get(i)) {
                return false;
            }
        }
        return true;
    }

    public void flickAllOrdered() {
        for (Slot slot : slots) {
            slot.flick();
        }
    }

    public void resetShovels() {
        for (Slot slot : slots) {
            slot.home();
        }
    }

    private static class Slot {
        private static final double SHOVEL_HOME = 0.05;
        private static final double SHOVEL_FLICK = 0.75;

        private final ColorSensor sensorA;
        private final ColorSensor sensorB;
        private final Servo shovel;

        Slot(ColorSensor sensorA, ColorSensor sensorB, Servo shovel) {
            this.sensorA = sensorA;
            this.sensorB = sensorB;
            this.shovel = shovel;
            home();
        }

        SortingPattern.ColorTarget classify() {
            int red = sensorA.red() + sensorB.red();
            int blue = sensorA.blue() + sensorB.blue();
            int green = sensorA.green() + sensorB.green();

            // Basic heuristic: green dominance = green, otherwise purple.
            if (green > red && green > blue) {
                return SortingPattern.ColorTarget.GREEN;
            }
            return SortingPattern.ColorTarget.PURPLE;
        }

        void flick() {
            shovel.setPosition(SHOVEL_FLICK);
        }

        void home() {
            shovel.setPosition(SHOVEL_HOME);
        }
    }
}
