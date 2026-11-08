package org.firstinspires.ftc.teamcode.RRAutos.Red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.RobotGeometry;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous (name = "Red: Hive Down Park")
public class RedHiveDown extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = RobotGeometry.pivotFromCenter(new Pose2d(-61, -12, Math.toRadians(180)));

        TankDrive drive = new TankDrive(hardwareMap, beginPose);
        
        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .setReversed(true).splineTo(new Vector2d(-48, -59), Math.toRadians(270))
                .build();

        Actions.runBlocking(new SequentialAction(path));

    }
}
