package org.firstinspires.ftc.teamcode.RRAutos.Red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous (name = "RoadRunner Starter Bot Auto")
public class Red2CellLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(61, -12, 0);

        TankDrive drive = new TankDrive(hardwareMap, beginPose);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                //shoot
                .lineToX(42)
                .turnTo(Math.toRadians(180))
                .splineTo(new Vector2d(36, -51), Math.toRadians(270))
                //start intaking
                .splineTo(new Vector2d(59.5, -61), Math.toRadians(0))
                .setReversed(true).splineTo(new Vector2d(24, -55), Math.toRadians(180))
                //top intaking
                .setReversed(true).splineTo(new Vector2d(-36, -11), Math.toRadians(90))
                .turn(Math.toRadians(-90))
                .lineToX(-61)
                //shoot
                .setReversed(true).splineTo(new Vector2d(-48, -59), Math.toRadians(270))
                //end here maybe if thats nessicary
                .build();

        Actions.runBlocking(new SequentialAction(path));

    }
}
