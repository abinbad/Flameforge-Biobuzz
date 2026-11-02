package org.firstinspires.ftc.teamcode.flywheeltest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "flywheelSlop", group = "Tuning")
public class flywheelSlop extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {

    private DcMotorEx launcher;
    double targetVelocity = 1300; // ticks/sec
    double p = 0.0;
    double f = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to tune PIDF");
        telemetry.update();

        waitForStart();

        // Step 1: Tune F automatically
        tuneF();

        // Step 2: Tune P automatically
        tuneP();

        // Print optimal values at the end of tuning
        telemetry.clearAll();
        telemetry.addLine("=== TUNING COMPLETE ===");
        telemetry.addData("Optimal P", p);
        telemetry.addData("Optimal F", f);
        telemetry.update();
        sleep(3000);

        // Step 3: Allow driver to change target velocity
        while (opModeIsActive()) {
            handleVelocityChange();
            launcher.setVelocity(targetVelocity);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", launcher.getVelocity());
            telemetry.addData("P", p);
            telemetry.addData("F", f);
            telemetry.update();
        }
    }

    private void tuneF() {
        telemetry.addLine("Tuning F (Feedforward)...");
        telemetry.update();

        // Test running at full power to find max velocity, then calculate F based on targetVelocity
        launcher.setPower(1.0);
        sleep(1200); // allow flywheel to reach max speed
        double maxVelocity = launcher.getVelocity();
        launcher.setPower(0);

        if (maxVelocity <= 0) {
            maxVelocity = targetVelocity * 2;
        }

        // Base feedforward F
        f = 1.0 / maxVelocity;

        // Test F by running at targetVelocity and measuring performance
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(0, 0, 0, f));
        launcher.setVelocity(targetVelocity);
        sleep(600);

        double measuredVel = launcher.getVelocity();
        // If actual velocity is below target velocity, scale F up to account for friction / load
        if (measuredVel < targetVelocity && measuredVel > 0) {
            f = f * (targetVelocity / measuredVel);
        }

        telemetry.addLine("F tuned");
        telemetry.addData("Max Velocity", maxVelocity);
        telemetry.addData("Tested F", f);
        telemetry.update();
        sleep(400);
    }

    private void tuneP() {
        telemetry.addLine("Tuning P...");
        telemetry.update();

        // Test a robust range of candidate P values considering target velocity response
        double[] testPValues = {0.001, 0.005, 0.01, 0.02, 0.05, 0.1, 0.15, 0.2};
        double bestP = testPValues[0];
        double minError = Double.MAX_VALUE;

        for (double testP : testPValues) {
            if (!opModeIsActive()) break;

            p = testP;
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new com.qualcomm.robotcore.hardware.PIDFCoefficients(p, 0, 0, f));

            launcher.setVelocity(targetVelocity);
            sleep(300); // test the response against target velocity

            double currentVel = launcher.getVelocity();
            double error = Math.abs(targetVelocity - currentVel);

            telemetry.addData("Testing P", p);
            telemetry.addData("Current Vel", currentVel);
            telemetry.addData("Target Vel", targetVelocity);
            telemetry.addData("Error", error);
            telemetry.update();

            if (error < minError) {
                minError = error;
                bestP = p;
            }
        }

        p = bestP;
        // Apply optimal tested P and F
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(p, 0, 0, f));
        launcher.setVelocity(targetVelocity);
        sleep(400);

        telemetry.addLine("P tuned");
        telemetry.addData("Optimal P", p);
        telemetry.update();
        sleep(400);
    }

    private void handleVelocityChange() {
        if (gamepad1.dpad_up) {
            targetVelocity += 100;
            sleep(200); // debounce
        }
        if (gamepad1.dpad_down) {
            targetVelocity -= 100;
            sleep(200); // debounce
        }
    }
}
