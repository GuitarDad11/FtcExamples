/*
 * DcMotorController
 *
 * A position-based PID controller for a single drive motor.
 * Converts encoder counts to inches for intuitive target setting.
 *
 * Usage:
 *   DcMotorController controller = new DcMotorController(hardwareMap, "left_drive",
 *                                                 countsPerInch, 1.0, 0.0, 0.0);
 *   controller.setTargetPosition(36.0);  // inches
 *
 *   // In your loop:
 *   controller.update();
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DcMotorController {

    /*----- Hardware -----*/
    private final DcMotor motor;

    /*----- Encoder conversion -----*/
    // Encoder counts per inch of travel.
    // Derived: (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER * Math.PI)
    private  double countsPerInch;

    /*----- PID gains -----*/
    private double kp;
    private double ki;
    private double kd;

    /*----- PID state -----*/
    private double targetPositionInches;
    private double integral;
    private double previousError;
    private double currentPower;
    /*----- Output limit -----*/
    private static final double MAX_POWER = 1.0;

    /**
     * Create a new PID controller for the given motor.
     *
     * @param hardwareMap       Active HardwareMap from the OpMode.
     * @param motorName         Motor name as configured in the Device Configuration Manager.
     * @param kp                Proportional gain.
     * @param ki                Integral gain (often 0 for simple drive control).
     * @param kd                Derivative gain.
     * @param countsPerInch     Encoder counts per inch of wheel travel.
     */
    public DcMotorController(HardwareMap hardwareMap,
                             String motorName,
                             double kp,
                             double ki,
                             double kd,
                             double countsPerInch) {
        this.motor = hardwareMap.get(DcMotor.class, motorName);
        this.setParameters(kp,ki,kd,countsPerInch);
        this.resetState();
    }

    /**
     * Set the desired position in inches (relative to the last encoder reset).
     *
     * @param inches  Target position in inches (positive = forward).
     */
    public void setTargetPosition(double inches) {
        this.targetPositionInches = inches;
    }

    /**
     * Set the desired power level
     *
     * @param power  new calculated power for motor.
     */
    private void setPower(double power) {
        // Clamp to valid motor power range.
        this.currentPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        // Send command to motor
        this.motor.setPower(this.currentPower);
    }

    /**
     * Set the PID gains at runtime (for tuning).
     *
     * @param kp  Proportional gain.
     * @param ki  Integral gain.
     * @param kd  Derivative gain.
     */
    public void setParameters(double kp, double ki, double kd,double countsPerInch) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.countsPerInch = countsPerInch;
    }

    /**
     * Compute and set the motor power based on the current encoder position.
     * Call this method each loop iteration.
     */
    public void update() {
        // Calculate how much farther to move
        double error = this.targetPositionInches - this.getCurrentPositionInches();
        // Integral: accumulate error over time.
        integral += error;
        // Derivative: rate of change of error.
        double derivative = error - this.previousError;
        this.previousError = error;
        // Update motor command
        this.setPower( kp * error
                    + ki * integral
                    + kd * derivative);
    }

    /**
     * @return  The underlying DcMotor (for telemetry, direction, etc.).
     */
    public DcMotor getMotor() {
        return this.motor;
    }

    /**
     * @return  Current position in inches.
     */
    public double getCurrentPositionInches() {
        return this.motor.getCurrentPosition() / countsPerInch;
    }
    /**
     * @return  Target position in inches.
     */
    public double getTargetPositionInches() {
        return this.targetPositionInches;
    }

    /**
     * Reset the integral and previous error (useful when changing targets).
     */
    public void resetState() {
        this.integral = 0.0;
        this.previousError = 0.0;
        this.targetPositionInches = 0.0;
        this.currentPower = 0.0;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setPower(0);
    }
}
