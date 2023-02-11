package org.firstinspires.ftc.teamcode.Comp.Libraries;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Interfaces.IDrive;
import org.firstinspires.ftc.teamcode.Interfaces.IGyro;

public class DriveWithoutEncoders implements IDrive {
    private PIDController _PIDDrive;
    private DcMotor _WheelFrontLeft;
    private DcMotor _WheelFrontRight;
    private DcMotor _WheelBackLeft;
    private DcMotor _WheelBackRight;
    private DcMotor _OdometerLeft;
    private DcMotor _OdometerRight;
    private Telemetry _Telemetry;
    private double _power = 0.3;
    private double _distance;
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

    public DriveWithoutEncoders(
            PIDController pidDrive,
            IGyro gyro,
            DcMotor WheelFrontLeft,
            DcMotor WheelFrontRight,
            DcMotor WheelBackLeft,
            DcMotor WheelBackRight,
            DcMotor OdometerLeft,
            DcMotor OdometerRight,
            Telemetry telemetry) {

        _PIDDrive = pidDrive;
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

        _distance = Math.abs(distanceInch);  // Encoder values decrease driving forward, hence a negative target distance.
        _power = Math.abs(power);            // Power will be negative driving forward.
        ResetEncoders();

        // Update current and target distances
        UpdateCurrentPositions();
        UpdateTargetPositions(distanceInch);

        // Set up parameters for driving in a straight line.
        ResetPIDDrive();

        do { // Drive until we reach the target distance
            _correction = -_PIDDrive.performPID(_Gyro.GetHeadingEH());

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
        } while (!onTarget());
        // !onTarget() = _leftCurrentPosition > _leftTargetPosition && _rightCurrentPosition > _rightTargetPosition
        // Current and target positions will be negative driving backward; current position will decrease the farther we drive.

        StopRobot();

        ShowTelemetry();

        sleep(_MILLS_TO_SLEEP);
    }

    @Override
    public void Backward(double distanceInch, double power) {
        sleep(100);

        _distance = -Math.abs(distanceInch);  // Encoder values Increase driving backward, hence a positive target distance.
        _power = -Math.abs(power);            // Power will be positive driving backward.
        ResetEncoders();

        // Update current and target distances
        UpdateCurrentPositions();
        UpdateTargetPositions(distanceInch);

        // Set up parameters for driving in a straight line.
        ResetPIDDrive();

        ShowTelemetry();

        do { // Drive until we reach the target distance
            _correction = -_PIDDrive.performPID(_Gyro.GetHeadingEH());

            ShowTelemetry();

            // TODO: We may need this.
//            if (_power - _correction <= 0.2) {
//                _power += _correction;
//            }

            _WheelBackLeft.setPower(_power + _correction);
            _WheelBackRight.setPower(_power - _correction);
            _WheelFrontLeft.setPower(_power + _correction);
            _WheelFrontRight.setPower(_power - _correction);

            UpdateCurrentPositions();
        } while (!onTarget());
        // !onTarget() = _leftCurrentPosition < _leftTargetPosition && _rightCurrentPosition < _rightTargetPosition
        // Current and target positions will be positive driving forward; current position will increase the farther we drive.

        StopRobot();

        ShowTelemetry();

        sleep(_MILLS_TO_SLEEP);
    }

    @Override
    public void Straight(DriveWithEncoders.Direction direction, double distanceInch, double power) {
        // Do nothing
    }

    @Override
    public void ShowTelemetry() {
        _Telemetry.addData("PID", "--- PID Information ---");
        _Telemetry.addData("power", _power);
        _Telemetry.addData("distance", _distance);
        _Telemetry.addData("heading", _Gyro.GetHeadingEH());
        _Telemetry.addData("target heading", _PIDDrive.getSetpoint());
        _Telemetry.addData("correction", _correction);

        _Telemetry.addData("OD", "--- Odometer Information ---");
        _Telemetry.addData("CL", "Current Left  : " + _leftCurrentPosition);
        _Telemetry.addData("TL", "Target Left: " + _leftTargetPosition);
        _Telemetry.addData("CR", "Current Right  : " + _rightCurrentPosition);
        _Telemetry.addData("TR", "Target Right: " + _rightTargetPosition);
        _Telemetry.update();
    }

    public double CalculateTicksPerInch(double ticksPerRevolution, double wheelDiameter) { // TODO: Test This
        TICKS_PER_INCH = ticksPerRevolution/(wheelDiameter * Math.PI);
        return TICKS_PER_INCH;
    }

    private void ResetEncoders() {
        _OdometerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _OdometerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _OdometerLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _OdometerRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        _leftCurrentPosition = _OdometerLeft.getCurrentPosition();
        _rightCurrentPosition = _OdometerRight.getCurrentPosition();

        _leftTargetPosition = 0;
        _rightTargetPosition = 0;
    }

    private void ResetPIDDrive(){
        _PIDDrive.reset();

        double p = 0.05;

        // Integrative factor can be approximated by diving P by 100. Then you have to tune
        // this value until the robot turns, slows down and stops accurately and also does
        // not take too long to "home" in on the setpoint.
//        double i = p / 175.0; // TODO: Try and find an integral that works well with driving straight.

        _PIDDrive.setPID(p, 0, 0);

        _PIDDrive.setSetpoint(0);
        _PIDDrive.setOutputRange(0, _power);
        _PIDDrive.setInputRange(-90, 90);
        _PIDDrive.setContinuous(true);
        _PIDDrive.enable();
    }

    private boolean onTarget() {
        return (Math.abs(_leftCurrentPosition) > Math.abs(_leftTargetPosition) || Math.abs(_rightCurrentPosition) > Math.abs(_rightTargetPosition));
    }

    // turn the motors off.
    private void StopRobot() {
        _WheelFrontLeft.setPower(0);
        _WheelBackLeft.setPower(0);
        _WheelFrontRight.setPower(0);
        _WheelBackRight.setPower(0);
    }

    private void UpdateCurrentPositions() {
        // Get Current positions in ticks
        _leftCurrentPosition = _OdometerLeft.getCurrentPosition();
        _rightCurrentPosition = _OdometerRight.getCurrentPosition();
    }

    private void UpdateTargetPositions(double distance) {
        // Get Target positions by adding current position and # of ticks to travel
        _leftTargetPosition = _leftCurrentPosition + CalcDistance(distance);
        _rightTargetPosition = _rightCurrentPosition + CalcDistance(distance);
    }

    private double CalcDistance(double distanceInch) { return TICKS_PER_INCH * distanceInch; }

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
