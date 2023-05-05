package org.firstinspires.ftc.teamcode.Christian;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Timmi TeleOp", group = "Competition")
public class TimmiTeleop extends OpMode {

    /*
     * Declare Hardware
     */

    // Wheels
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

    // Claw
    private Servo Claw;
    private boolean clawIsOpen = true;
    private boolean buttonClawIsPressed = false;
    private double clawPosition = 0.0;
    private final double CLAW_OPEN = 0.0;
    private final double CLAW_CLOSED = 0.55;

    // SlowMode Drive
    private double percentToSlowDrive = 1.0;


    @Override
    public void init() {

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

        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Initialize Claw
        telemetry.addData("I", "Initializing Claw");
        telemetry.update();

        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setDirection(Servo.Direction.FORWARD);
        ZeroClaw();


        // Let the user know initialization is complete.
        telemetry.addData("I", "Initialization Complete! :D");
        telemetry.update();

    }

    @Override
    public void loop() {

        /*
         * Gamepad Controls
         */

        // Gamepad 1
        double oneLeftStickYPower = -gamepad1.left_stick_y;
        double oneLeftStickXPower = gamepad1.left_stick_x;
        double oneRightStickXPower = gamepad1.right_stick_x;
        boolean oneButtonA = gamepad1.a;

        /*
         * Do Stuff Here!
         */

        // Drive Controls
        ProMotorControl(oneLeftStickYPower, oneLeftStickXPower, oneRightStickXPower);

        // Claw Controls
        ToggleClaw(oneButtonA);

        telemetry.update();
    }


    /*
     * Methods
     */

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    //******************************************************************
    // Get the inputs from the controller for power [ PRO ]
    //******************************************************************
    private void ProMotorControl(double left_stick_y, double left_stick_x, double right_stick_x) {
        double powerLeftY = -left_stick_y;   // DRIVE : Backward -1 <---> 1 Forward
        double powerLeftX = left_stick_x*-1; // STRAFE:     Left -1 <---> 1 Right
        double powerRightX = -right_stick_x; // ROTATE:     Left -1 <---> 1 Right

        double r = Math.hypot(powerLeftX, powerLeftY);
        double robotAngle = Math.atan2(powerLeftY, powerLeftX) - Math.PI / 4;
        double leftX = powerRightX;
        final double v1 = r * Math.cos(robotAngle) / Math.sin(Math.PI/4) + leftX;
        final double v2 = r * Math.sin(robotAngle) / Math.sin(Math.PI/4) - leftX;
        final double v3 = r * Math.sin(robotAngle) / Math.sin(Math.PI/4) + leftX;
        final double v4 = r * Math.cos(robotAngle) / Math.sin(Math.PI/4) - leftX;

        WheelFrontLeft.setPower(v1* percentToSlowDrive);
        WheelFrontRight.setPower(v2* percentToSlowDrive);
        WheelBackLeft.setPower(v3* percentToSlowDrive);
        WheelBackRight.setPower(v4* percentToSlowDrive);

        telemetry.addData("Wheel Front Left",v1* percentToSlowDrive);
        telemetry.addData("Wheel Front Right",v2* percentToSlowDrive);
        telemetry.addData("Wheel Back Left",v3* percentToSlowDrive);
        telemetry.addData("Wheel Back Right",v4* percentToSlowDrive);
    }

    private void ToggleClaw(boolean button) {
        if (button && !buttonClawIsPressed) {
            buttonClawIsPressed = true;
            clawIsOpen = !clawIsOpen;
        } if (!button) { buttonClawIsPressed = false; }

        if (clawIsOpen) {
            OpenClaw();
            telemetry.addData("Claw Position","Open: " + CLAW_OPEN);
        } else {
            CloseClaw();
            telemetry.addData("Claw Position","Closed: " + CLAW_CLOSED);
        }

        Claw.setPosition(clawPosition);
    }

    private void ZeroClaw() { Claw.setPosition(0.0); }

    private void CloseClaw() { clawPosition = CLAW_CLOSED; }

    private void OpenClaw() { clawPosition = CLAW_OPEN; }
}
