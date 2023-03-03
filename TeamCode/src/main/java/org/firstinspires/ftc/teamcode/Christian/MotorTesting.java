package org.firstinspires.ftc.teamcode.Christian;

//<<<<<<< HEAD
////import com.acmerobotics.roadrunner.drive.Drive;
//=======
//>>>>>>> e92c857614ba0430d8cdd18257964cbfe4d5256a
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous(name = "Motor Testing", group = "Lessons")
public class MotorTesting extends LinearOpMode {

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


    @Override
    public void runOpMode() throws InterruptedException {

        Initialize();
        waitForStart();

        while (opModeIsActive()) {

            /*
             * Do Stuff Here!
             */

            WheelFrontRight.setPower(0.5);
            sleep(5000);
            WheelFrontRight.setPower(0.0);

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


        // Let the user know initialization is complete.
        telemetry.addData("I", "Initialization Complete! :D");
        telemetry.update();
    }
}