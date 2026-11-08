package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Drawing {
    private Drawing() {}

    /** Draws the real chassis rectangle (offset from the pivot) plus a dot at the pivot. */
    public static void drawRobot(Canvas c, Pose2d t) {
        final double front = RobotGeometry.FRONT_EXTENT_IN;
        final double rear  = RobotGeometry.REAR_EXTENT_IN;
        final double half  = RobotGeometry.ROBOT_WIDTH_IN / 2.0;

        // corners in the robot frame (x forward, y left), relative to the pivot
        double[][] local = {{front, half}, {front, -half}, {-rear, -half}, {-rear, half}};
        double[] xs = new double[4], ys = new double[4];
        for (int i = 0; i < 4; i++) {
            Vector2d p = t.position.plus(t.heading.times(new Vector2d(local[i][0], local[i][1])));
            xs[i] = p.x;
            ys[i] = p.y;
        }

        c.setStrokeWidth(1);
        c.strokePolygon(xs, ys);

        // pivot dot + heading tick
        c.fillCircle(t.position.x, t.position.y, 1.0);
        Vector2d nose = t.position.plus(t.heading.vec().times(front));
        c.strokeLine(t.position.x, t.position.y, nose.x, nose.y);
    }
}
