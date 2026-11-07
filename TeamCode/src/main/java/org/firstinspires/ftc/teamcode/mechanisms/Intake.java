package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends InterruptedException {
    private final DcMotor intake;
    private final CRServo rightIntakeServo;
    private final CRServo leftIntakeServo;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");


    }

    // ---------- Direct control (TeleOp / anywhere) ----------
    public void setIntakePower(double power) { intake.setPower(power); }
    public void setIntakeServoPower(double power) {
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
        intake.setPower(0);
        leftIntakeServo.setPower(0);
    }

    // ---------- Road Runner Actions (Auto) ----------
    public Action stopIntake() {
        return new InstantAction(this::stop);
    }

    public Action startIntake() {
        return new InstantAction(() -> {
            setIntakePower(1);
            setIntakeServoPower(1);
        });
    }
}
