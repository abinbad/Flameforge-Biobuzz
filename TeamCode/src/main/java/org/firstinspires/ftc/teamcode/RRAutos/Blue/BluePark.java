package org.firstinspires.ftc.teamcode.RRAutos.Blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotGeometry;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

@Autonomous (name = "Blue: Simple Park")
public class BluePark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = RobotGeometry.pivotFromCenter(new Pose2d(-61, 12, Math.toRadians(180)));

        TankDrive drive = new TankDrive(hardwareMap, beginPose);
        
        Launcher shoot = new Launcher(hardwareMap);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(shoot.close_fire())
                .setReversed(true).splineTo(new Vector2d(0, 48), Math.toRadians(0))
                .setReversed(true).splineTo(new Vector2d(24, 59), Math.toRadians(90))
                .build();

        Actions.runBlocking(new SequentialAction(path));

    }
}
