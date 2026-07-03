/*
 * OpMode: Servo Control
 * Controls a single servo using the gamepad.
 *
 * Controls:
 *   Left  Trigger  - Increase servo position
 *   Right Trigger  - Decrease servo position
 *   A / X button   - Set servo to minimum (0.0)
 *   B / Y button   - Set servo to maximum (1.0)
 *
 * The current servo position is displayed on the telemetry.
 *
 * Setup:
 *   1. Attach a servo to a servo port on your controller/expansion hub.
 *   2. In the Device Configuration Manager, name the servo "my_servo".
 *   3. Select this OpMode from the Driver Station and press Play.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Control", group = "TeleOp")
public class ServoControl extends LinearOpMode {
    static final double SERVO_MIN = 0.0;
    static final double SERVO_MAX = 1.0;
    static final double SERVO_STEP = 0.03;
    static final int UPDATE_INTERVAL_MS = 50;
    double position = 0.5;  // Start at midpoint

    @Override
    public void runOpMode() {
        // Initialize servo named "servo0"
        double position = 0.5;
        Servo servo = hardwareMap.get(Servo.class, "servo0");
        servo.setPosition(position);
        // Display status and wait for the start button
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        // Main control loop
        while (opModeIsActive()) {
            // Read triggers (0.0 to 1.0)
            float leftTrigger  = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;

            // Increase position with right trigger
            if (rightTrigger > 0.1) {
                position = Math.max(servo.getPosition() + SERVO_STEP, SERVO_MAX);
            }

            // Decrease position with left trigger
            if (leftTrigger > 0.1) {
                position = Math.min(servo.getPosition() - SERVO_STEP, SERVO_MIN);
            }

            // A / X button sets to minimum
            if (gamepad1.a || gamepad1.x) {
                position = SERVO_MIN;
            }

            // B / Y button sets to maximum
            if (gamepad1.b || gamepad1.y) {
                position = SERVO_MAX;
            }

            // Apply the new position
            servo.setPosition(position);
            // Update telemetry
            telemetry.addData("Servo Position", "%.2f", position);
            telemetry.update();
            sleep(UPDATE_INTERVAL_MS);
        }
    }
}