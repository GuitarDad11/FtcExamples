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

@Autonomous(name = "Auto Square Drive", group = "Autonomous")
public class AutoSquareDrive extends LinearOpMode {

    /*----- Hardware references -----*/
    private DcMotor leftDrive  = null;
    private DcMotor rightDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    /*----- Encoder math -----*/
    // Encoder counts per full motor revolution (check your motor datasheet).
    static final double COUNTS_PER_MOTOR_REV  = 1440.0;

    // Gear reduction between motor and wheel (1.0 = direct drive).
    // Example: 12T motor gear driving a 24T wheel gear = 2.0.
    static final double DRIVE_GEAR_REDUCTION  = 1.0;

    // Drive wheel diameter in inches.
    static final double WHEEL_DIAMETER_INCHES = 4.0;

    // Derived: encoder counts per inch of wheel travel.
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                                                / (WHEEL_DIAMETER_INCHES * Math.PI);

    /*----- Movement parameters -----*/
    // How far to drive on each side of the square (36 inches = 3 feet).
    static final double DRIVE_DISTANCE_INCHES = 36.0;

    // Per-wheel travel for a 90-degree right turn.
    // Left wheel moves forward, right wheel moves backward (pivot turn).
    // Tune this value so the robot turns exactly 90 degrees.
    // Start with approximately half the wheelbase in inches, then adjust.
    static final double TURN_DISTANCE_INCHES  = 12.0;

    // Motor power (0.0 - 1.0). Lower values are more precise.
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED  = 0.4;

    // Timeout in seconds for each move (safety net).
    static final double MOVE_TIMEOUT_S = 5.0;

    /*----- Square pattern -----*/
    static final int SIDES = 4;

    @Override
    public void runOpMode() {

        /*----- Initialize motors -----*/
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need one motor reversed so both sticks forward = robot goes forward.
        // Swap these directions if the robot moves backward when you expect forward.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders to zero.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Switch to encoder-driven mode (RUN_TO_POSITION ready).
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Report encoder positions.
        telemetry.addData("Starting encoders", "left=%7d  right=%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();

        /*----- Wait for driver to press START -----*/
        waitForStart();
        runtime.reset();

        /*----- Drive the square -----*/
        if (opModeIsActive()) {
            for (int side = 1; side <= SIDES; side++) {
                telemetry.addData("Side", "%d / %d", side, SIDES);
                telemetry.update();

                // Step 1: Drive forward.
                encoderDrive(DRIVE_SPEED,
                        DRIVE_DISTANCE_INCHES,  DRIVE_DISTANCE_INCHES,
                        MOVE_TIMEOUT_S);

                // Step 2: Turn right 90 degrees.
                encoderDrive(TURN_SPEED,
                        TURN_DISTANCE_INCHES, -TURN_DISTANCE_INCHES,
                        MOVE_TIMEOUT_S);
            }
        }

        /*----- Done -----*/
        telemetry.addData("Path", "Square complete");
        telemetry.addData("Time", "%.1f seconds", runtime.seconds());
        telemetry.update();
        sleep(1000);
    }

    /*
     * Drive the robot a specified distance (in inches) on each side.
     * Positive inches = forward, negative = reverse.
     *
     * @param speed       Motor power (0.0-1.0); sign is ignored (absolute value used).
     * @param leftInches  Distance for the left wheel (positive = forward).
     * @param rightInches Distance for the right wheel (positive = forward).
     * @param timeoutS    Maximum seconds to wait before giving up.
     */
    public void encoderDrive(double speed,
                             double leftInches,
                             double rightInches,
                             double timeoutS) {

        if (!opModeIsActive()) {
            return;
        }

        // Calculate absolute encoder targets from current position.
        int newLeftTarget  = leftDrive.getCurrentPosition()
                + (int) (leftInches * COUNTS_PER_INCH);
        int newRightTarget = rightDrive.getCurrentPosition()
                + (int) (rightInches * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // Enable RUN_TO_POSITION mode.
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the move.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        // Wait until both motors reach their target, timeout expires, or op mode stops.
        while (opModeIsActive()
                && runtime.seconds() < timeoutS
                && leftDrive.isBusy()
                && rightDrive.isBusy()) {

            telemetry.addData("Running to",  "L=%7d  R=%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Currently at", "L=%7d  R=%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors.
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Return to encoder mode (not RUN_TO_POSITION) so the next call sets a fresh target.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brief settle pause.
        sleep(250);
    }
}
