package org.firstinspires.ftc.teamcode.RRAutos.Blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Launcher;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous (name = "RoadRunner Starter Bot Auto")
public class Blue2CellRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-61, 12, 180);

        TankDrive drive = new TankDrive(hardwareMap, beginPose);
        
        Launcher shoot = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(shoot.close_fire())
                 .lineToX(-42)
                 .turnTo(Math.toRadians(0))
                 .splineTo(new Vector2d(-36, 51), Math.toRadians(90))
                 .stopAndAdd(intake.startIntake())
                 .splineTo(new Vector2d(-59.5, 61), Math.toRadians(180))
                 .setReversed(true).splineTo(new Vector2d(-24, 55), Math.toRadians(0))
                .stopAndAdd(intake.stopIntake())
                 .setReversed(true).splineTo(new Vector2d(36, 11), Math.toRadians(270))
                 .turn(Math.toRadians(-90))
                 .lineToX(61)
                 .stopAndAdd(shoot.close_fire())
                 .setReversed(true).splineTo(new Vector2d(24, 59), Math.toRadians(90))
                 //end here maybe if thats nessicary
                .build();

        Actions.runBlocking(new SequentialAction(path));

    }
}
