package org.firstinspires.ftc.teamcode.flywheeltest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Auto PIDF Tuner", group = "Tuning")
public class flywheelSlop extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {

    DcMotorEx flywheel;
    double targetVelocity = 1500; // ticks/sec
    double tolerance = 50;        // acceptable error in ticks/sec
    double stepSizeP = 0.0005;    // small increments for P tuning
    double p = 0.0;
    double f = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to tune PIDF");
        telemetry.update();

        waitForStart();

        // Step 1: Tune F automatically
        tuneF();

        // Step 2: Tune P automatically
        tuneP();

        // Step 3: Allow driver to change target velocity
        while (opModeIsActive()) {
            handleVelocityChange();
            flywheel.setVelocity(targetVelocity);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", flywheel.getVelocity());
            telemetry.addData("P", p);
            telemetry.addData("F", f);
            telemetry.update();
        }
    }

    private void tuneF() {
        // Apply a fixed power and measure steady-state velocity
        flywheel.setPower(0.5);
        sleep(2000); // wait for steady state
        double measuredVelocity = flywheel.getVelocity();

        // Calculate F gain (power per velocity)
        f = 0.5 / measuredVelocity;
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(p, 0, 0, f));

        telemetry.addLine("F tuned");
        telemetry.addData("F", f);
        telemetry.update();
    }

    private void tuneP() {
        flywheel.setVelocity(targetVelocity);
        sleep(1000);

        double error = targetVelocity - flywheel.getVelocity();
        while (Math.abs(error) > tolerance && opModeIsActive()) {
            p += stepSizeP * (error / targetVelocity);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new com.qualcomm.robotcore.hardware.PIDFCoefficients(p, 0, 0, f));

            flywheel.setVelocity(targetVelocity);
            sleep(500);
            error = targetVelocity - flywheel.getVelocity();

            telemetry.addData("Tuning P", p);
            telemetry.addData("Error", error);
            telemetry.update();
        }
        telemetry.addLine("P tuned");
        telemetry.update();
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
