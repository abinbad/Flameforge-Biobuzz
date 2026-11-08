package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TankDrive;

/**
 * Commands ONE full turn (360 deg) with all feedback turned off, and measures how far the robot
 * really turned by adding up the IMU yaw during the whole turn (so a 90 deg turn reads as 90,
 * not as a small error). Also adds up the odopod heading so you can compare the two.
 *
 * Restore trackWidthTicks to your known-good value before running this.
 */
@Autonomous(name = "TrackWidthCheck", group = "tuning")
public class TrackWidthCheck extends LinearOpMode {
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
        try {
            // pure open loop: no gains, no settle phase
            TankDrive.PARAMS.turnGain = 0;
            TankDrive.PARAMS.turnVelGain = 0;
            TankDrive.PARAMS.turnSettleTimeout = 0;

            Action turn = drive.actionBuilder(new Pose2d(0, 0, 0)).turn(2 * Math.PI).build();
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
            telemetry.addData("Commanded", "360 deg");
            telemetry.addData("IMU says the robot turned", "%.1f deg", imuDeg);
            telemetry.addData("Odopods say it turned", "%.1f deg", locDeg);
            if (Math.abs(imuDeg - locDeg) > 5) {
                telemetry.addLine("Odopod heading disagrees with the IMU -> re-run the angular ramp");
            }
            if (Math.abs(imuDeg) > 10) {
                telemetry.addData("Turned / commanded", "%.0f%%", 100.0 * imuDeg / 360.0);
                telemetry.addLine("Well under 100%: raise TankDrive.PARAMS.kTurn (try +0.5 V at a time).");
                telemetry.addLine("Over 100%: lower kTurn. Leave trackWidthTicks alone unless it is");
                telemetry.addLine("far from the real wheel-to-wheel distance.");
            }
            telemetry.update();
        }
    }
}
