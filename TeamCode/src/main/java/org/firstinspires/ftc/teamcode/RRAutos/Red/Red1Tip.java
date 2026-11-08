package org.firstinspires.ftc.teamcode.RRAutos.Red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.RobotGeometry;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous (name = "Auto")
public class Red1Tip extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = RobotGeometry.pivotFromCenter(new Pose2d(61, -12, Math.toRadians(0)));

        TankDrive drive = new TankDrive(hardwareMap, beginPose);
        
        Launcher shoot = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(shoot.close_fire())
                 .lineToX(36)
                 .turn(Math.toRadians(-90))
                 .splineTo(new Vector2d(32, -42), Math.toRadians(270), new TranslationalVelConstraint(20))
                .splineTo(new Vector2d(46,-59.5), Math.toRadians(0))
                 .stopAndAdd(intake.startIntake())
                .lineToX(61)
                .waitSeconds(.75)
                 .stopAndAdd(intake.stopIntake())
                .setReversed(true).splineTo(new Vector2d(30,-47), Math.toRadians(150))
                .setReversed(false).splineTo(new Vector2d(50, -48), Math.toRadians(320),  new TranslationalVelConstraint(20))
                .stopAndAdd(shoot.soft_fire())
                .setReversed(true).splineTo(new Vector2d(0,-42), Math.toRadians(180))
                .splineTo(new Vector2d(-24, -59), Math.toRadians(270), new TranslationalVelConstraint(60))
                .build();

        Actions.runBlocking(new SequentialAction(path));

    }

}
