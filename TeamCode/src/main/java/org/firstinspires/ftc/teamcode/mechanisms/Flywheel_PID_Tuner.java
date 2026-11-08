package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Config
@TeleOp(name = "Flywheel PID Tuner")
public class Flywheel_PID_Tuner extends OpMode {
    public DcMotorEx flywheelMotor;

    public static double highVelocity = 1500;
    public static double lowVelocity = 1250;
    public static double P = 105.0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 12.4;

    private double curTargetVelocity = highVelocity;
    private boolean motorsRunning = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double loopTime = 0.0;

    private double maxError = 0;
    private double avgError = 0;
    private int sampleCount = 0;
    private double errorSum = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        updatePIDFCoefficients();

        telemetry.addLine("=== Flywheel PID Tuner ===");
        telemetry.addLine("Configurable via FTC Dashboard");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("X: Start/Stop Motor");
        telemetry.addLine("Y: Toggle High/Low Velocity");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            motorsRunning = !motorsRunning;
            if (motorsRunning) {
                runtime.reset();
                resetMetrics();
            } else {
                flywheelMotor.setVelocity(0);
            }
            sleep(200);
        }

        if (gamepad1.y) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
            resetMetrics();
            sleep(200);
        }

        updatePIDFCoefficients();

        if (motorsRunning) {
            flywheelMotor.setVelocity(curTargetVelocity);
        }

        displayTelemetry();
    }

    private void updatePIDFCoefficients() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    private void resetMetrics() {
        maxError = 0;
        avgError = 0;
        sampleCount = 0;
        errorSum = 0;
    }

    private void displayTelemetry() {
        double vel = flywheelMotor.getVelocity();
        double error = Math.abs(curTargetVelocity - vel);
        double errorPercent = (error / curTargetVelocity) * 100.0;

        if (motorsRunning) {
            errorSum += error;
            sampleCount++;
            avgError = errorSum / sampleCount;
            maxError = Math.max(maxError, error);
        }

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", vel);
        telemetry.addData("Error", error);
        telemetry.addData("Error Percent", errorPercent);

        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);

        telemetry.addData("Max Error", maxError);
        telemetry.addData("Avg Error", avgError);
        telemetry.addData("Motors Running", motorsRunning);
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Loop Hz", getHz());

        String stability = errorPercent < 1 ? "EXCELLENT" :
                errorPercent < 2 ? "GOOD" :
                        errorPercent < 5 ? "ACCEPTABLE" : "POOR";
        telemetry.addData("Stability", stability);

        telemetry.update();
    }

    public double getHz() {
        double loop = System.nanoTime();
        double hz = 1000000000 / (loop - loopTime);
        loopTime = loop;
        return hz;
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
