/*
 * OpMode: Auto Square Drive
 * Drives the robot in a square pattern using encoder-based positioning.
 *
 * Behavior:
 *   Repeats 4 times:
 *     1. Drive forward for a set distance (36 inches / 3 feet)
 *     2. Turn right 90 degrees
 *
 * Requirements:
 *   - Motors with encoders configured in the Device Configuration Manager.
 *   - Motor names must match the hardware configuration (default: "left_drive", "right_drive").
 *   - Positive motor power must move the robot forward and cause encoders to count UP.
 *
 * Tuning:
 *   - COUNTS_PER_MOTOR_REV  : Encoder counts per full motor rotation (Tetrix = 1440).
 *   - WHEEL_DIAMETER_INCHES : Diameter of your drive wheels.
 *   - DRIVE_DISTANCE_INCHES : How far to travel on each side of the square.
 *   - TURN_DISTANCE_INCHES  : Left/right wheel travel for a 90-degree turn.
 *                              Positive left + negative right = right turn.
 *                              Tune by measuring how much each wheel travels during a 90-degree pivot.
 *   - DRIVE_SPEED / TURN_SPEED : Motor power (0.0 to 1.0). Lower = more precise.
 *
 * Setup:
 *   1. Configure drive motors in the FTC Robot Controller app.
 *   2. Remove @Disabled to enable in the Driver Station.
 *   3. Select "Auto Square Drive" from the autonomous op mode list.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Drive Example", group = "Autonomous")
public class AutoDriveExample extends LinearOpMode {

    /*----- Hardware references -----*/
    private DcMotorController rightDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*----- Initialize motors -----*/
        rightDrive = new DcMotorController(
                hardwareMap,
                "right_drive",
                1,
                1,
                1,
                1);
        
        /*----- Wait for driver to press START -----*/
        waitForStart();
        runtime.reset();

        /*----- Drive the square -----*/
        if (opModeIsActive()) {
            for (int side = 1; side <= 4; side++) {
                // Step 1: Drive forward.
                encoderDrive(DRIVE_SPEED, DRIVE_DISTANCE_INCHES,
                        MOVE_TIMEOUT_S);

                // Step 2: Turn right 90 degrees.
                encoderDrive(TURN_SPEED,-TURN_DISTANCE_INCHES,
                        MOVE_TIMEOUT_S);
            }
        }

        /*----- Done -----*/
        telemetry.addData("Elapsed Time", "%.1f seconds", runtime.seconds());
        telemetry.update();
    }

    /*
     * Drive the robot a specified distance (in inches) on each side.
     * Positive inches = forward, negative = reverse.
     *
     * @param speed       Motor power (0.0-1.0); sign is ignored (absolute value used).
     * @param rightInches Distance for the right wheel (positive = forward).
     * @param timeoutS    Maximum seconds to wait before giving up.
     */
    public void encoderDrive(double speed,
                             double rightInches,
                             double timeoutS) {

        if (!opModeIsActive()) {
            return;
        }

        // Calculate absolute encoder targets from current position.
        int newRightTarget = rightDrive.getCurrentPosition()
                + (int) (rightInches * COUNTS_PER_INCH);

        rightDrive.setTargetPosition(newRightTarget);

        // Enable RUN_TO_POSITION mode.
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the move.
        runtime.reset();
        rightDrive.setPower(Math.abs(speed));

        // Wait until both motors reach their target, timeout expires, or op mode stops.
        while (opModeIsActive()
                && runtime.seconds() < timeoutS
                && rightDrive.isBusy()) {

            telemetry.addData("Running to",  "R=%7d", newRightTarget);
            telemetry.addData("Currently at", "R=%7d",
                    rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors.
        rightDrive.setPower(0);

        // Return to encoder mode (not RUN_TO_POSITION) so the next call sets a fresh target.
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brief settle pause.
        sleep(250);
    }
}
