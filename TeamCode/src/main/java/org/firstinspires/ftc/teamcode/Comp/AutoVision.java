package org.firstinspires.ftc.teamcode.Comp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//@Disabled
@Autonomous(name = "Auto Vision", group = "Competition")
public class AutoVision extends LinearOpMode {

    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

//    private IVision _Vision;
    private SignalZone signalIcon;

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private enum SignalZone {
        LEFT,
        CENTER,
        RIGHT,
        NULL
    }

    private float bestConfidence = 0;
    private String bestLabel = "null";

    private static final String VUFORIA_KEY =
            "AXbaM4X/////AAABmasKHKdK60IXjDtokRJeu6w/6UeS3KpD2vdyQEvji2tDLEy+IToNeCC4oU0iEAOEzsgw8FABI0qMvJ001KiAfvt3YQETOyjFMY++rqydjE49FLsEPSMGzch3gzelSF9gw3gusCb0rhP/GGXaZnYpN4HbYYI9o/7jUgenQTxlblDFwDsjSgf8TiIoJGTMsW77RCv90nhsWlD+i8qYEUwM3pCxlQ0jImn1+uTTQfoLRNJEn1ZCrDaTcjf5+yxsgdHDXyB5Xh9hd031YFVjX8nX+m9n1ZDHAp8Ha3nLH1MYM5TUuh2/CNZMgyw2BPpAaasW4hT9aDiaYKAVlHQ32dVlTIie2Za4gVFGHgHaahZyUMuz";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize Wheels
        telemetry.addData("I", "Initializing Wheels");
        telemetry.update();

//        WheelFrontLeft = hardwareMap.dcMotor.get("WheelFL");
//        WheelFrontRight = hardwareMap.dcMotor.get("WheelFR");
//        WheelBackLeft = hardwareMap.dcMotor.get("WheelBL");
//        WheelBackRight = hardwareMap.dcMotor.get("WheelBR");
//
//        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        WheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        WheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        WheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        WheelFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        WheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        WheelBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        WheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        _Vision = new VisionDefault();

        // _Vision.turnOnCamera();
        turnOnCamera();




        waitForStart();

        while (opModeIsActive()) {      // If we've stopped the robot, stop the program

            signalIcon = locateSignalIcon();

//            ProMotorControl(-0.1, 0.0, 0.0);  // Backward because the robot is backward
//            sleep(1500);

            switch(signalIcon) {
                case LEFT:
                    telemetry.addData("Direction", "Left!");
//                    ProMotorControl(0, 0.1, 0); // Strafe right because the robot is backward
                    break;
                case RIGHT:
                    telemetry.addData("Direction", "Right!");
//                    ProMotorControl(0, -0.1, 0); // Strafe left because the robot is backward
                    break;
                case CENTER:
                    telemetry.addData("Direction", "Center!");
                    // Stay Still
                    break;
                default:
                    telemetry.addData("Direction", "Invalid Direction!");
                    break;
            }
            telemetry.update();
            sleep(100);



//            ProMotorControl(0.0, 0.0, 0.0);  // Stop
//            break; // End the program once it has finished
        }

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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.50f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private void turnOnCamera() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }

    private SignalZone locateSignalIcon() {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        do {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() > 0) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getConfidence() >= bestConfidence) {
                                bestConfidence = recognition.getConfidence();
                                bestLabel = recognition.getLabel();
                            }
                        }
                    }
                }
            }
        } while (timer.milliseconds() <= 1500);

        return returnSignalZone(bestLabel);
    }

    private SignalZone returnSignalZone(String signalIcon) {
        if (signalIcon == LABELS[0]) {
            return SignalZone.LEFT;
        } else if (signalIcon == LABELS[1]) {
            return SignalZone.CENTER;
        } else if (signalIcon == LABELS[2]) {
            return SignalZone.RIGHT;
        } else {
            return SignalZone.NULL;
        }
    }

}
