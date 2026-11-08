package org.firstinspires.ftc.teamcode.RRAutos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.RobotGeometry;
import org.firstinspires.ftc.teamcode.tuning.TankDrive;


@Disabled
@Autonomous (name = "Auto")
public class Turn extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = RobotGeometry.pivotFromCenter(new Pose2d(0, 0, Math.toRadians(0)));

        TankDrive drive = new TankDrive(hardwareMap, beginPose);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .turn(Math.toRadians(90))
                .waitSeconds(2)
                .turn(Math.toRadians(150))
                .waitSeconds(2)
                .turn(Math.toRadians(330))
                .build();

        Actions.runBlocking(new SequentialAction(path));

    }
}
