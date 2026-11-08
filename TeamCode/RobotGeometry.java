package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Road Runner's pose for a tank drive is the PIVOT: the point midway between the two
 * powered wheels. The numbers below only describe the robot's outline relative to that
 * point. They are used for start poses, wall clearance and drawing. They do not affect
 * how the robot is controlled.
 *
 * Measure all three with a tape measure, to the outermost edge of anything you want
 * counted (frame, bumpers, intake, mechanisms that stick out).
 */
public final class RobotGeometry {
    private RobotGeometry() {}

    // TODO: measure (inches)
    /** Pivot -> front-most edge of the robot, along the heading direction. */
    public static final double FRONT_EXTENT_IN = 11.5;
    /** Pivot -> rear-most edge of the robot. */
    public static final double REAR_EXTENT_IN  = 6.0;
    /** Full side-to-side width of the robot. */
    public static final double ROBOT_WIDTH_IN  = 15.0;

    // Derived
    public static final double ROBOT_LENGTH_IN = FRONT_EXTENT_IN + REAR_EXTENT_IN;
    /** How far the middle of the outline is ahead of the pivot (2.75 with the numbers above). */
    public static final double CENTER_AHEAD_OF_PIVOT_IN = (FRONT_EXTENT_IN - REAR_EXTENT_IN) / 2.0;

    /** You know where the middle of the outline is -> pose Road Runner should use (the pivot). */
    public static Pose2d pivotFromCenter(Pose2d center) {
        Vector2d off = center.heading.times(new Vector2d(-CENTER_AHEAD_OF_PIVOT_IN, 0));
        return new Pose2d(center.position.plus(off), center.heading);
    }

    /** Road Runner pose (pivot) -> where the middle of the outline is. */
    public static Pose2d centerFromPivot(Pose2d pivot) {
        Vector2d off = pivot.heading.times(new Vector2d(CENTER_AHEAD_OF_PIVOT_IN, 0));
        return new Pose2d(pivot.position.plus(off), pivot.heading);
    }
}
