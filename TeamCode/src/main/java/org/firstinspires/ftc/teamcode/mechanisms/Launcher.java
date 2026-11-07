package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Launcher extends InterruptedException {
    private final DcMotorEx launcher;
    private final CRServo windmill;

    public static double CLOSE_TARGET_VELOCITY = 1275;
    public static double CLOSE_MINIMUM_VELOCITY = 1225;
    public static double WINDMILL_POWER = 1;
    public static double feedTime = 5;
    public static double spinUpTimeout = 3;

    public Launcher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmill = hardwareMap.get(CRServo.class, "windmill");

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(105, 0, 0, 12.4));
        windmill.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ---------- Direct control (TeleOp / anywhere) ----------
    public void setLauncherPower(double power) { launcher.setPower(power); }
    public void setWindmillPower(double power) { windmill.setPower(power); }

    public void stop() {
        launcher.setPower(0);
        windmill.setPower(0);
    }

    // ---------- Road Runner Actions (Auto) ----------
    public Action stopAll() {
        return new InstantAction(this::stop);
    }

    /** Starts the flywheel, then keeps running until it reaches minVelocity (or times out). */
    private Action spinUpAndWait(double target, double minVelocity) {
        return new Action() {
            private boolean started = false;
            private double startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double now = System.nanoTime() / 1e9;
                if (!started) {
                    launcher.setVelocity(target);
                    startTime = now;
                    started = true;
                }

                double velocity = Math.abs(launcher.getVelocity());
                packet.put("launcher velocity", velocity);

                boolean atSpeed = velocity >= minVelocity;
                boolean timedOut = (now - startTime) > spinUpTimeout;

                // true = "not finished, call me again", false = "done"
                return !(atSpeed || timedOut);
            }
        };
    }

    public Action close_fire() {
        return new SequentialAction(
                spinUpAndWait(CLOSE_TARGET_VELOCITY, CLOSE_MINIMUM_VELOCITY),
                new InstantAction(() -> windmill.setPower(WINDMILL_POWER)),
                new SleepAction(feedTime),
                stopAll(),
                new SleepAction(1.5)
        );
    }
}
