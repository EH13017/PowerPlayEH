package org.firstinspires.ftc.teamcode.Comp.Libraries;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.IDrive;
import org.firstinspires.ftc.teamcode.Interfaces.IGyro;
import org.firstinspires.ftc.teamcode.Comp.Libraries.PIDController;

public class DriveWithEncoders implements IDrive {
    private PIDController _PIDDriveDistance;
    private PIDController _PIDDriveStraight;
    private DcMotor _WheelFrontLeft;
    private DcMotor _WheelFrontRight;
    private DcMotor _WheelBackLeft;
    private DcMotor _WheelBackRight;
    private DcMotor _OdometerLeft;
    private DcMotor _OdometerRight;
    private Telemetry _Telemetry;
    private double _power = 0.3;
    private double _targetDistance;
    private IGyro _Gyro;

    private double _correction;
    private double _leftCurrentPosition = 0;
    private double _leftTargetPosition = 0;
    private double _rightCurrentPosition = 0;
    private double _rightTargetPosition = 0;

    private final long _MILLS_TO_SLEEP = 1000;

    private final double TICKS_PER_REVOLUTION = 537.6;
    private final double WHEEL_DIAMETER_INCHES = 4;
    private final double INCHES_PER_REVOLUTION = WHEEL_DIAMETER_INCHES * Math.PI;
    private double TICKS_PER_INCH = TICKS_PER_REVOLUTION/INCHES_PER_REVOLUTION;
    private final double DEGREES_PER_TICK = 360/TICKS_PER_REVOLUTION;

    public DriveWithEncoders(
            PIDController pidDriveDistance,
            PIDController pidDriveStraight,
            IGyro gyro,
            DcMotor WheelFrontLeft,
            DcMotor WheelFrontRight,
            DcMotor WheelBackLeft,
            DcMotor WheelBackRight,
            DcMotor OdometerLeft,
            DcMotor OdometerRight,
            Telemetry telemetry) {

        _PIDDriveDistance = pidDriveDistance;
        _PIDDriveStraight = pidDriveStraight;
        _WheelBackLeft = WheelBackLeft;
        _WheelBackRight = WheelBackRight;
        _WheelFrontLeft = WheelFrontLeft;
        _WheelFrontRight = WheelFrontRight;
        _OdometerLeft = OdometerLeft;
        _OdometerRight = OdometerRight;
        _Gyro = gyro;
        _Telemetry = telemetry;
    }

    @Override
    public void Forward(double distanceInch, double power) {
        sleep(100);

        _targetDistance = -Math.abs(InchesToDegrees(distanceInch)); // Encoder values decrease driving forward, hence a negative target distance.
        _power = -Math.abs(power);                                  // Power will be negative driving forward.
        ResetEncoders();

        // Update current and target distances
        UpdateCurrentPositions();
        UpdateTargetPositions();

        // Set up parameters for driving in a straight line.
        ResetPIDDriveDistance();
        ResetPIDDriveStraight();

        do { // Drive until we reach the target distance
            UpdateCurrentPositions();
            _power = _PIDDriveDistance.performPID(_leftCurrentPosition);
            _correction = _PIDDriveStraight.performPID(_Gyro.GetHeadingEH());

            ShowTelemetry();

            // TODO: We may need this.
//            if (_power + _correction >= -0.2) {
//                _power -= _correction;
//            }

            _WheelBackLeft.setPower(_power + _correction);
            _WheelBackRight.setPower(_power - _correction);
            _WheelFrontLeft.setPower(_power + _correction);
            _WheelFrontRight.setPower(_power - _correction);
            UpdateCurrentPositions();
        } while (!_PIDDriveDistance.onTarget());

        StopRobot();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _Gyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }

    @Override
    public void Backward(double distanceInch, double power) {
        sleep(100);

        _targetDistance = Math.abs(InchesToDegrees(distanceInch)); // Encoder values Increase driving backward, hence a positive target distance.
        _power = Math.abs(power);                                  // Power will be positive driving backward.
        ResetEncoders();

        // Update current and target distances
        UpdateCurrentPositions();
        UpdateTargetPositions();

        // Set up parameters for driving in a straight line.
        ResetPIDDriveDistance();
        ResetPIDDriveStraight();

        ShowTelemetry();

        do { // Drive until we reach the target distance
            UpdateCurrentPositions();
            _power = _PIDDriveDistance.performPID(_leftCurrentPosition);
            _correction = _PIDDriveStraight.performPID(_Gyro.GetHeadingEH());

            ShowTelemetry();

            // TODO: We may need this.
//            if (_power + _correction >= -0.2) {
//                _power -= _correction;
//            }

            _WheelBackLeft.setPower(_power + _correction);
            _WheelBackRight.setPower(_power - _correction);
            _WheelFrontLeft.setPower(_power + _correction);
            _WheelFrontRight.setPower(_power - _correction);

            UpdateCurrentPositions();
        } while (!_PIDDriveDistance.onTarget());

        StopRobot();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _Gyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }

    public void Straight(Direction direction, double distanceInch, double power) {
        sleep(100);

        int sign;
        switch (direction) {
            case FORWARD:
                sign = 1; // Encoder values Decrease driving forward, hence a negative target distance.
                break;
            case BACKWARD:
                sign = -1;  // Encoder values Increase driving backward, hence a positive target distance.
                break;
            default:
                sign = 0;  // If we enter in the wrong direction, the robot won't move.
                break;

        }

        _targetDistance = Math.abs(InchesToDegrees(distanceInch)) * sign;
        _power = Math.abs(power) * sign;  // Power will be positive driving backward, and negative driving forward.
        ResetEncoders();

        // Update current and target distances
        UpdateCurrentPositions();
        UpdateTargetPositions();

        // Set up parameters for driving in a straight line.
        //ResetPIDDriveDistance();
        ResetPIDDriveStraight();

        ShowTelemetry();

        do { // Drive until we reach the target distance
            UpdateCurrentPositions();
            //_power = _PIDDriveDistance.performPID(_leftCurrentPosition);
            _correction = -_PIDDriveStraight.performPID(_Gyro.GetHeadingEH());

            ShowTelemetry();

            // TODO: We may need this.
//            if (_power + _correction >= -0.2) {
//                _power -= _correction;
//            }

            _WheelBackLeft.setPower(_power + _correction);
            _WheelBackRight.setPower(_power - _correction);
            _WheelFrontLeft.setPower(_power + _correction);
            _WheelFrontRight.setPower(_power - _correction);
            //UpdateCurrentPositions();
        } while (!_PIDDriveStraight.onTarget());

        StopRobot();

        ShowTelemetry();

        // reset angle tracking on new heading.
        _Gyro.ResetHeadingEH();

        sleep(_MILLS_TO_SLEEP);
    }

    @Override
    public void ShowTelemetry() {
        _Telemetry.addData("PID", "--- PID Information ---");
        _Telemetry.addData("power", _power);
        _Telemetry.addData("target distance", _targetDistance);

        _Telemetry.addData("PID", "--- DriveDistance Information ---");
        _Telemetry.addData("CL", "Current Left  : " + _leftCurrentPosition);
        _Telemetry.addData("TL", "Target Left: " + _leftTargetPosition);
        _Telemetry.addData("CR", "Current Right  : " + _rightCurrentPosition);
        _Telemetry.addData("TR", "Target Right: " + _rightTargetPosition);
        _Telemetry.addData("target distance", _PIDDriveDistance.getSetpoint());
        _Telemetry.addData("error", _PIDDriveDistance.getError());

        _Telemetry.addData("PID", "--- DriveStraight Information ---");
        _Telemetry.addData("heading", _Gyro.GetHeadingEH());
        _Telemetry.addData("target heading", _PIDDriveStraight.getSetpoint());
        _Telemetry.addData("correction", _correction);

        _Telemetry.addData("OD", "--- Odometer Information ---");
        _Telemetry.update();
    }

    private boolean onTarget() {
        return (Math.abs(_leftCurrentPosition) > Math.abs(_leftTargetPosition) || Math.abs(_rightCurrentPosition) > Math.abs(_rightTargetPosition));
    }

    private void StopRobot() {
        _WheelFrontLeft.setPower(0);
        _WheelBackLeft.setPower(0);
        _WheelFrontRight.setPower(0);
        _WheelBackRight.setPower(0);
    }

    private void UpdateCurrentPositions() {
        // Get Current positions in degrees
        _leftCurrentPosition = TicksToDegrees(_OdometerLeft.getCurrentPosition());
        //_rightCurrentPosition = TicksToDegrees(_OdometerRight.getCurrentPosition());
    }

    private void UpdateTargetPositions() {
        // Get Target positions by adding current position and # of degrees to travel
        _leftTargetPosition = _leftCurrentPosition + _targetDistance;
        //_rightTargetPosition = _rightCurrentPosition + _targetDistance;
    }

    private void ResetEncoders() {
        _OdometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //_OdometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _OdometerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //_OdometerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _leftCurrentPosition = TicksToDegrees(_OdometerLeft.getCurrentPosition());
        //_rightCurrentPosition = TicksToDegrees(_OdometerRight.getCurrentPosition());

        _leftTargetPosition = 0;
        _rightTargetPosition = 0;
    }

    private double CalcDistanceDegrees(double distanceInch) { return (TICKS_PER_INCH * distanceInch)/ DEGREES_PER_TICK; }

    private double InchesToTicks(double inches) {
        return inches * TICKS_PER_INCH;
    }

    private double TicksToDegrees(double ticks) {
        return ticks * DEGREES_PER_TICK;
    }

    private double InchesToDegrees(double inches) {
        return inches * TICKS_PER_INCH * DEGREES_PER_TICK;
    }

    private void ResetPIDDriveDistance(){

        _PIDDriveDistance.reset();

        // Proportional factor can be found by dividing the max desired pid output by
        // the setpoint or target. Here 30% power is divided by 90 degrees (.30 / 90)
        // to get a P factor of .003. This works for the robot we testing this code with.
        // Your robot may vary but this way finding P works well in most situations.
        double p = Math.abs(_power/ _targetDistance);

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint.
        double i = p / 100; // TODO: Figure this out

        _PIDDriveDistance.setPID(p, i, 0);

        _PIDDriveDistance.setInputRange(0, _targetDistance);
        _PIDDriveDistance.setSetpoint(_targetDistance);
        _PIDDriveDistance.setOutputRange(0.1, _power);
        _PIDDriveDistance.setTolerance(1.0 / Math.abs(_targetDistance) * 100.0); // One degree as a percentage of the total degrees
        _PIDDriveDistance.enable();
    }

    private void ResetPIDDriveStraight(){
        _PIDDriveStraight.reset();

        double p = 0.04;

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint.
//        double i = p / 175.0; // TODO: Try and find an integral that works well with driving straight.

        _PIDDriveStraight.setPID(p, 0, 0);

        _PIDDriveStraight.setSetpoint(0);
        _PIDDriveStraight.setOutputRange(0, _power);
        _PIDDriveStraight.setInputRange(-90, 90);
        _PIDDriveStraight.setContinuous(true);
        _PIDDriveStraight.enable();
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
}
