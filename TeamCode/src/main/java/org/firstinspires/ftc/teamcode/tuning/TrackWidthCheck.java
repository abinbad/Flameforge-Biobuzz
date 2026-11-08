package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TankDrive;

/**
 * Turns the robot by ANGLE_DEG and reports how far it REALLY turned (IMU, summed over the whole
 * move) next to what the odopods think.
 *
 *  CLOSED_LOOP = false : pure feedforward (gains 0, no settle phase). Tests the model only.
 *  CLOSED_LOOP = true  : exactly what turn() does in an auto.
 *
 * Change ANGLE_DEG / CLOSED_LOOP in FTC Dashboard before pressing play.
 */
@Disabled
@Config
@Autonomous(name = "TrackWidthCheck", group = "tuning")
public class TrackWidthCheck extends LinearOpMode {
    public static double ANGLE_DEG = 90;
    public static boolean CLOSED_LOOP = false;

    private static double imuYaw(TankDrive d) {
        return d.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

        double g = TankDrive.PARAMS.turnGain, vg = TankDrive.PARAMS.turnVelGain,
                st = TankDrive.PARAMS.turnSettleTimeout;
        double imuTotal = 0, locTotal = 0;

        waitForStart();
        final double angleDeg = ANGLE_DEG;
        final boolean closed = CLOSED_LOOP;
        try {
            if (!closed) {
                TankDrive.PARAMS.turnGain = 0;
                TankDrive.PARAMS.turnVelGain = 0;
                TankDrive.PARAMS.turnSettleTimeout = 0;
            }

            Action turn = drive.actionBuilder(new Pose2d(0, 0, 0)).turn(Math.toRadians(angleDeg)).build();
            TelemetryPacket packet = new TelemetryPacket();

            drive.updatePoseEstimate();
            double lastImu = imuYaw(drive);
            double lastLoc = drive.localizer.getPose().heading.toDouble();

            boolean running = true;
            ElapsedTime after = null;
            while (opModeIsActive() && (running || after.seconds() < 1.0)) {
                if (running) {
                    running = turn.run(packet);
                    if (!running) after = new ElapsedTime();
                } else {
                    drive.updatePoseEstimate();
                }
                double imu = imuYaw(drive);
                double loc = drive.localizer.getPose().heading.toDouble();
                imuTotal += AngleUnit.normalizeRadians(imu - lastImu);
                locTotal += AngleUnit.normalizeRadians(loc - lastLoc);
                lastImu = imu;
                lastLoc = loc;
            }
        } finally {
            TankDrive.PARAMS.turnGain = g;
            TankDrive.PARAMS.turnVelGain = vg;
            TankDrive.PARAMS.turnSettleTimeout = st;
        }

        double imuDeg = Math.toDegrees(imuTotal);
        double locDeg = Math.toDegrees(locTotal);

        while (opModeIsActive()) {
            telemetry.addData("Mode", closed ? "closed loop (like an auto)" : "feedforward only");
            telemetry.addData("Commanded", "%.0f deg", angleDeg);
            telemetry.addData("IMU says it turned", "%.1f deg", imuDeg);
            telemetry.addData("Odopods say it turned", "%.1f deg", locDeg);
            if (Math.abs(imuDeg - locDeg) > 5) {
                telemetry.addLine("Odopods disagree with the IMU -> re-run the angular ramp");
            }
            telemetry.update();
        }
    }
}