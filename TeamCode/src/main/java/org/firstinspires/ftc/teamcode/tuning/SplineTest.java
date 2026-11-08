package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.unused.MecanumDrive;

@Disabled
public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        // Road Runner tracks the pivot (middle of the powered axle); start with the chassis center at the origin
        Pose2d tankBeginPose = RobotGeometry.pivotFromCenter(beginPose);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, tankBeginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(tankBeginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());

            // Where does the robot THINK it ended up? Compare with a tape measure on the floor.
            sleep(500);
            drive.updatePoseEstimate();
            Pose2d est = drive.localizer.getPose();
            Pose2d goal = new Pose2d(0, 60, Math.PI);
            double h = est.heading.toDouble();
            double dx = goal.position.x - est.position.x;
            double dy = goal.position.y - est.position.y;
            double forwardErr = dx * Math.cos(h) + dy * Math.sin(h); // + = short of goal, - = past it

            while (opModeIsActive()) {
                telemetry.addData("estimated x, y (in)", "%.2f, %.2f", est.position.x, est.position.y);
                telemetry.addData("goal x, y (in)", "0.00, 60.00");
                telemetry.addData("estimated heading (deg)", "%.1f (goal 180)", Math.toDegrees(h));
                telemetry.addData("forward error (in)", "%.2f", forwardErr);
                telemetry.addLine("Now measure where the robot really is. Estimate ~ goal but robot past it");
                telemetry.addLine("= localizer scale problem. Estimate past the goal = controller problem.");
                telemetry.update();
            }
        } else {
            throw new RuntimeException();
        }
    }
}