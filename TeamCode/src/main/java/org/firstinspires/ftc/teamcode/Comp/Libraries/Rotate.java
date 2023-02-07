package org.firstinspires.ftc.teamcode.Comp.Libraries;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.IGyro;
import org.firstinspires.ftc.teamcode.Interfaces.IRotate;

public class Rotate implements IRotate {
    private PIDController _PIDRotate;
    private DcMotor _WheelFrontLeft;
    private DcMotor _WheelFrontRight;
    private DcMotor _WheelBackLeft;
    private DcMotor _WheelBackRight;
    private Telemetry _Telemetry;
    private double _power;
    private int _degrees;
    private IGyro _Gyro;

    private final long _MILLS_TO_SLEEP = 1000;

    public Rotate(
            PIDController pidRotate,
            IGyro gyro,
            DcMotor WheelFrontLeft,
            DcMotor WheelFrontRight,
            DcMotor WheelBackLeft,
            DcMotor WheelBackRight,
            Telemetry telemetry) {

        _PIDRotate = pidRotate;
        _WheelBackLeft = WheelBackLeft;
        _WheelBackRight = WheelBackRight;
        _WheelFrontLeft = WheelFrontLeft;
        _WheelFrontRight = WheelFrontRight;
        _Gyro = gyro;
        _Telemetry = telemetry;
    }

    /* Counterclockwise (left) = positive heading
     * Clockwise (right) = negative heading */

    /**
     * Rotate right the number of degrees.
     *
     * @param degrees Degrees to turn; negative degrees still rotates right.
     */
    @Override
    public void Left(int degrees, double power) {
        sleep(100);
        _degrees = CompensateForError(Math.abs(degrees)); // Left turn is positive degrees
        _power = Math.abs(power);     // Given power will always be positive
        _Gyro.ResetHeadingEH();

        ResetPIDRotate();

        ShowTelemetry();

        do {
            _power = _PIDRotate.performPID(_Gyro.GetHeadingEH()); // power will be negative on a right turn
            ShowTelemetry();
            _WheelFrontLeft.setPower(-_power);
            _WheelBackLeft.setPower(-_power);
            _WheelFrontRight.setPower(_power);
            _WheelBackRight.setPower(_power);
        } while (!_PIDRotate.onTarget());

        StopRotation();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _Gyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }

    /**
     * Rotate left the number of degrees.
     *
     * @param degrees Degrees to turn; negative degrees still rotates left.
     */
    @Override
    public void Right(int degrees, double power) {
        sleep(100);
        _degrees = -CompensateForError(Math.abs(degrees)); // Right turn is negative degrees
        _power = Math.abs(power);      // Given power will always be positive
        _Gyro.ResetHeadingEH();

        ResetPIDRotate();

        ShowTelemetry();

        do {
            _power = _PIDRotate.performPID(_Gyro.GetHeadingEH());
            ShowTelemetry();
            _WheelFrontLeft.setPower(-_power);
            _WheelBackLeft.setPower(-_power);
            _WheelFrontRight.setPower(_power);
            _WheelBackRight.setPower(_power);
        } while (!_PIDRotate.onTarget());

        StopRotation();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _Gyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }


    @Override
    public void ShowTelemetry() {
        _Telemetry.addData("power", _power);
        _Telemetry.addData("degrees", _degrees);
        _Telemetry.addData("heading", _Gyro.GetHeadingEH());
        _Telemetry.addData("target heading", _PIDRotate.getSetpoint());
        _Telemetry.addData("error", _PIDRotate.getError());
        _Telemetry.update();
    }

    // turn the motors off.
    private void StopRotation() {
        _WheelFrontLeft.setPower(0);
        _WheelBackLeft.setPower(0);
        _WheelFrontRight.setPower(0);
        _WheelBackRight.setPower(0);
    }

    private int CompensateForError(int degrees) {
        double degreesD = (double) degrees;
        double degreesCompensated = (degreesD - degreesD*0.055555555555555556);
        int degreesI = (int) Math.round(degreesCompensated);
        return degreesI;
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    private final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /* Start pid controller. PID controller will monitor the turn angle with respect to the
     * target angle and reduce power as we approach the target angle. This is to prevent the
     * robots momentum from overshooting the turn after we turn off the power. The PID controller
     * reports onTarget() = true when the difference between turn angle and target angle is within
     * 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
     * dependant on the motor and gearing configuration, starting power, weight of the robot and the
     * on target tolerance. If the controller overshoots, it will reverse the sign of the output
     * turning the robot back toward the set point value. */
    private void ResetPIDRotate(){

        _PIDRotate.reset();

        // Proportional factor can be found by dividing the max desired pid output by
        // the setpoint or target. Here 30% power is divided by 90 degrees (.30 / 90)
        // to get a P factor of .003. This works for the robot we testing this code with.
        // Your robot may vary but this way finding P works well in most situations.
        double p = Math.abs(_power/_degrees);

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint.
        double i = p / 100;

        _PIDRotate.setPID(p, i, 0);

        _PIDRotate.setInputRange(0, _degrees);
        _PIDRotate.setSetpoint(_degrees);
        _PIDRotate.setOutputRange(0.1, _power); // TODO: May need to change minimum output
//        _PIDRotate.setTolerance(2.0 / Math.abs(_degrees) * 100.0); // Two degrees as a percentage of the total degrees
        _PIDRotate.setTolerance(1.0 / Math.abs(_degrees) * 100.0); // One degree as a percentage of the total degrees
//        _PIDRotate.setTolerance(0.5 / Math.abs(_degrees) * 100.0); // Half degree as a percentage of the total degrees
        _PIDRotate.enable();
    }
}
