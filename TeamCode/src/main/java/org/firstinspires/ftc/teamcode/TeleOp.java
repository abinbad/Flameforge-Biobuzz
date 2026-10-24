package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mechanum Drive", group = "TeleOp")
public class TeleOp extends LinearOpMode {

    private MechanumDrive drive = new MechanumDrive();

    @Override
    public void runOpMode() {

        drive.init(hardwareMap);

        // Starting speed
        double maxSpeed = 0.3;

        // Used to detect a new button press
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        telemetry.addLine("Mechanum Drive Ready");
        telemetry.addData("Speed", "%.1f", maxSpeed);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Check if D-pad UP was newly pressed
            if (gamepad1.dpad_up && !lastDpadUp) {
                maxSpeed += 0.1;
            }

            // Check if D-pad DOWN was newly pressed
            if (gamepad1.dpad_down && !lastDpadDown) {
                maxSpeed -= 0.1;
            }

            // Keep speed between 0.1 and 1.0
            if (maxSpeed > 1.0) {
                maxSpeed = 1.0;
            }

            if (maxSpeed < 0.1) {
                maxSpeed = 0.1;
            }

            // Remember the current button states
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // Controller:
            // Left stick Y = Forward / Backward
            // Left stick X = Strafe Left / Right
            // Right stick X = Rotate

            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Send the current speed to the mecanum drive
            drive.drive(forward, strafe, rotate, maxSpeed);

            telemetry.addData("Speed", "%.1f", maxSpeed);
            telemetry.update();
        }
    }
}
