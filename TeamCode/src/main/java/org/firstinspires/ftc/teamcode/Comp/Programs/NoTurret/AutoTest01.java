package org.firstinspires.ftc.teamcode.Comp.Programs.NoTurret;

//<<<<<<< HEAD
////import com.acmerobotics.roadrunner.drive.Drive;
//=======
//>>>>>>> e92c857614ba0430d8cdd18257964cbfe4d5256a
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Comp.Libraries.AprilTag;
import org.firstinspires.ftc.teamcode.Comp.Libraries.DriveWithoutEncoders;
import org.firstinspires.ftc.teamcode.Comp.Libraries.EHGyro;
import org.firstinspires.ftc.teamcode.Comp.Libraries.PIDController;
import org.firstinspires.ftc.teamcode.Comp.Libraries.Rotate;
import org.firstinspires.ftc.teamcode.Interfaces.IDrive;
import org.firstinspires.ftc.teamcode.Interfaces.IGyro;
import org.firstinspires.ftc.teamcode.Interfaces.IRotate;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name = "Auto Main", group = "Competition")
public class AutoTest01 extends LinearOpMode {

    /*
     * Declare Hardware
     */

    // Wheels
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;
    private double power = 0.4;
    private double PERCENT_TO_SLOW = 0.60;
    private double distanceToDrive = 4; // Distance after deciding which Signal Zone to park in

    // Encoders
    private DcMotor EncoderLeft;
    private DcMotor EncoderRight;

    // Code Libraries
    private IRotate _Rotate;
    private IGyro _EHGyro;
    private IDrive _Drive;
    public AprilTag _AprilTag;

    // Timer - A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();
    private final int DELAY_MILLS = 500;

    // PID
    private PIDController pidRotate, pidDriveDistance, pidDriveStraight;

    // Gyroscope
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;

//    // REV Blinkin
//    private RevBlinkinLedDriver LED;

    // AprilTag Vision
    OpenCvCamera camera;
    private int signalValue = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        Initialize();
        waitForStart();

        while (opModeIsActive()) {

            /*
             * Do Stuff Here!
             */

//            // Reset the LEDs
//            turnOffLEDPattern();

            // Detect the AprilTag on the Signal Sleeve
            timer.reset();
            do {
//                signalValue = 2; // Manual detection
                signalValue = _AprilTag.DetectTag(); // Automatic detection
            } while (timer.milliseconds() < 3000);


            // Drive into Signal Zone 2
            _Drive.Backward(22, power);
            sleep(DELAY_MILLS);


            // Determine which Signal Zone to park in
            if (signalValue == _AprilTag.LEFT) { // Left
                telemetry.addData("Signal Zone", "Left");
                telemetry.update();
                distanceToDrive = 23;
//                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                _Rotate.Left(90, power);
            }
            else if (signalValue == _AprilTag.MIDDLE) { // Middle
                telemetry.addData("Signal Zone", "Middle");
                telemetry.update();
                distanceToDrive = 4;
//                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                // Don't Rotate
            }
            else if (signalValue == _AprilTag.RIGHT) { // Right
                telemetry.addData("Signal Zone", "Right");
                telemetry.update();
                distanceToDrive = 28;
//                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                _Rotate.Right(90, power);
            }
            else { // Invalid
                telemetry.addData("Signal Zone", "Invalid");
                telemetry.update();
                distanceToDrive = 4;
//                setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

                // Don't Rotate - Invalid Zone
            }
            sleep(DELAY_MILLS);
//            turnOffLEDPattern();


            // Drive into the appropriate Signal Zone
            _Drive.Backward(distanceToDrive, power);


//            // Show off the pretty lights once we are finished!!
//            setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
//            sleep(5000);


            break; // End the program once it has finished
        }

    }


    /*
     * Methods
     */

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


        // Initialize Encoders
        telemetry.addData("I", "Initializing Encoders");
        telemetry.update();

        EncoderLeft = WheelFrontLeft;
        EncoderRight = WheelFrontRight;


//        // Initialize Lift TODO: Integrate Lift system
//        telemetry.addData("I", "Initializing Lift");
//        telemetry.update();


        // Initialize Gyro
        telemetry.addData("I", "Initializing Gyro and PID");
        telemetry.update();

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
        telemetry.addData("I", "Initializing Webcam");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        // Initialize Code Libraries
        telemetry.addData("I", "Initializing Code Libraries");
        telemetry.update();

        _EHGyro = new EHGyro(modernRoboticsI2cGyro);
        _EHGyro.ResetHeadingEH();
        _Rotate = new Rotate(pidRotate, _EHGyro, WheelFrontLeft, WheelFrontRight, WheelBackLeft, WheelBackRight, telemetry);
//        _Drive = new DriveWithEncoders(pidDriveDistance, pidDriveStraight, _EHGyro, WheelFrontLeft, WheelFrontRight, WheelBackLeft, WheelBackRight, OdometerLeft, OdometerRight, telemetry);
        _Drive = new DriveWithoutEncoders(pidDriveStraight, _EHGyro, WheelFrontLeft, WheelFrontRight, WheelBackLeft, WheelBackRight, EncoderLeft, EncoderRight, telemetry);
        _AprilTag = new AprilTag(camera, telemetry);


//        // Initialize REV Blinkin
//        telemetry.addData("I", "Initializing Blinkin");
//        telemetry.update();
//
//        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
//        setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


        // Let the user know initialization is complete.
        telemetry.addData("I", "Initialization Complete! :D");
        telemetry.update();
    }

//    // Here is a file to show how to use the REV Blinkin, along with a complete list of colors:
//    // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
//    protected void setLEDPattern(RevBlinkinLedDriver.BlinkinPattern setPattern) {
//        LED.setPattern(setPattern);
//    }
//
//    protected void turnOffLEDPattern() {
//        LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//    }


}