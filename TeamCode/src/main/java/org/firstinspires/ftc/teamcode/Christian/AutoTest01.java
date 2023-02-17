package org.firstinspires.ftc.teamcode.Christian;

//<<<<<<< HEAD
////import com.acmerobotics.roadrunner.drive.Drive;
//=======
//>>>>>>> e92c857614ba0430d8cdd18257964cbfe4d5256a
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Comp.Libraries.AprilTag;
import org.firstinspires.ftc.teamcode.Comp.Libraries.AprilTagAutonomousInitDetectionExample;
import org.firstinspires.ftc.teamcode.Comp.Libraries.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Comp.Libraries.DriveWithEncoders;
import org.firstinspires.ftc.teamcode.Comp.Libraries.DriveWithoutEncoders;
import org.firstinspires.ftc.teamcode.Comp.Libraries.EHGyro;
import org.firstinspires.ftc.teamcode.Comp.Libraries.PIDController;
import org.firstinspires.ftc.teamcode.Comp.Libraries.Rotate;
import org.firstinspires.ftc.teamcode.Interfaces.IAprilTag;
import org.firstinspires.ftc.teamcode.Interfaces.IDrive;
import org.firstinspires.ftc.teamcode.Interfaces.IGyro;
import org.firstinspires.ftc.teamcode.Interfaces.IRotate;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@Autonomous(name = "Auto Test 01", group = "Testing")
public class AutoTest01 extends LinearOpMode {

    private int delayMills = 500;

    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;
    private double PERCENT_TO_SLOW = 0.60;

    private double distanceToDrive = 4;

    // Encoders
    private DcMotor OdometerLeft;
    private DcMotor OdometerRight;

    private IRotate _Rotate;
    private IGyro _EHGyro;
    private IDrive _Drive;
    public AprilTag _AprilTag;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    // PID
    private PIDController pidRotate, pidDriveDistance, pidDriveStraight;

    // Gyroscope
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    private double power = 0.4;

    ColorSensor color;

    // REV Blinkin
    private RevBlinkinLedDriver LED;

    // AprilTag Vision
    OpenCvCamera camera;
    private int signalValue = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        Initialize();

        waitForStart();

        while (opModeIsActive()) {

            // Reset the LEDs
            turnOffLEDPattern();

            timer.reset();
            do {
//                signalValue = 2;
                signalValue = _AprilTag.DetectTag();
            } while (timer.milliseconds() < 3000);


            _Drive.Backward(22, power);

            sleep(delayMills);

            if (signalValue == _AprilTag.LEFT) { // Right
                telemetry.addData("Signal Zone", "Left");
                telemetry.update();
                distanceToDrive = 23;
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                _Rotate.Left(90, power); // Left
            } else if (signalValue == _AprilTag.MIDDLE) { // Middle
                telemetry.addData("Signal Zone", "Middle");
                telemetry.update();
                distanceToDrive = 4;
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                // Nothing - Middle
            } else if (signalValue == _AprilTag.RIGHT) { // Right
                telemetry.addData("Signal Zone", "Right");
                telemetry.update();
                distanceToDrive = 28;
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                _Rotate.Right(90, power); // Right
            } else { // invalid
                telemetry.addData("Signal Zone", "Invalid");
                telemetry.update();
                distanceToDrive = 4;
                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                // Nothing - Invalid
            }

            sleep(delayMills);
            turnOffLEDPattern();

            _Drive.Backward(distanceToDrive, power);

            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            sleep(5000);


            break; // End the program once it has finished
        }

    }

    public void Initialize() {

        // Initialize Wheels
        telemetry.addData("I", "Initializing Wheels");
        telemetry.update();

        WheelFrontLeft = hardwareMap.dcMotor.get("WheelFL");
        WheelFrontRight = hardwareMap.dcMotor.get("WheelFR");
        WheelBackLeft = hardwareMap.dcMotor.get("WheelBL");
        WheelBackRight = hardwareMap.dcMotor.get("WheelBR");

        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize Encoders TODO: Figure out how to code builtin odometers
        OdometerLeft = WheelFrontLeft;
        OdometerRight = WheelFrontRight;

        // TODO: Integrate Lift system

        // Initialize Gyro
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        telemetry.addData("GC", "Gyro Calibrating. Do Not Move!");
        telemetry.update();
        modernRoboticsI2cGyro.calibrate();

        // Initialize PID

        /* Set PID proportional value to start reducing power at about 50 degrees of rotation.
         * P by itself may stall before turn completed so we add a bit of I (integral) which
         * causes the PID controller to gently increase power if the turn is not completed. */
        pidRotate = new PIDController(0, 0, 0);

        /* Set PID proportional value to produce non-zero correction value when robot veers off
         * straight line. P value controls how sensitive the correction is. */
//        pidDriveDistance = new PIDController(0, 0, 0);
        pidDriveStraight = new PIDController(0.05, 0, 0);

        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        // Initialize AprilTag Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        _EHGyro = new EHGyro(modernRoboticsI2cGyro);
        _EHGyro.ResetHeadingEH();
        _Rotate = new Rotate(pidRotate, _EHGyro, WheelFrontLeft, WheelFrontRight, WheelBackLeft, WheelBackRight, telemetry);
//        _Drive = new DriveWithEncoders(pidDriveDistance, pidDriveStraight, _EHGyro, WheelFrontLeft, WheelFrontRight, WheelBackLeft, WheelBackRight, OdometerLeft, OdometerRight, telemetry);
        _Drive = new DriveWithoutEncoders(pidDriveStraight, _EHGyro, WheelFrontLeft, WheelFrontRight, WheelBackLeft, WheelBackRight, OdometerLeft, OdometerRight, telemetry);
        _AprilTag = new AprilTag(camera, telemetry);

        // REV Blinkin Initialization
        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        telemetry.addData("Initialization", "Complete!");
        telemetry.update();

    }

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    //******************************************************************
    // Get the inputs from the controller for power [ PRO ]
    //******************************************************************
    private void ProMotorControl(double right_stick_y, double right_stick_x, double left_stick_x) {
        double powerRightY = right_stick_y; // DRIVE : Backward -1 <---> 1 Forward
        double powerRightX = right_stick_x; // STRAFE:     Left -1 <---> 1 Right
        double powerLeftX = left_stick_x;   // ROTATE:     Left -1 <---> 1 Right

        double r = Math.hypot(powerRightX, powerRightY);
        double robotAngle = Math.atan2(powerRightY, powerRightX) - Math.PI / 4;
        double leftX = powerLeftX;
        final double v1 = r * Math.cos(robotAngle) + leftX;
        final double v2 = r * Math.sin(robotAngle) - leftX;
        final double v3 = r * Math.sin(robotAngle) + leftX;
        final double v4 = r * Math.cos(robotAngle) - leftX;

        WheelFrontLeft.setPower(v1);
        WheelFrontRight.setPower(v2);
        WheelBackLeft.setPower(v3);
        WheelBackRight.setPower(v4);
    }

    private void detectColor() {
        if(color.red()>=color.green() & color.red()>=color.blue()){
            telemetry.addData("Color","Red");
//            colorValue = 1;
            // Turn lights red
            //Left
        }
        if(color.green()>=color.red() & color.green()>=color.blue()){
            telemetry.addData("Color","Green");
//            colorValue = 2;
            // Turn lights green
            //Park
        }
        if(color.blue()>=color.red() & color.blue()>=color.green()){
            telemetry.addData("Color","Blue");
//            colorValue = 3;
            // Turn lights blue
            //Right
        }
    }

    // Here is a file to show how to use the REV Blinkin, along with a complete list of colors:
    // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
    protected void setLEDPattern(RevBlinkinLedDriver.BlinkinPattern setPattern) {
        LED.setPattern(setPattern);
    }

    protected void turnOffLEDPattern() {
        LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }


}