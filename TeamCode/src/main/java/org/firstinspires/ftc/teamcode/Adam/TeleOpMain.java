package org.firstinspires.ftc.teamcode.Adam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp Main", group = "Competition")
public class TeleOpMain extends OpMode {

    /*
     * Declare Hardware
     */

    // Wheels
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;


    private DcMotor Lift;

    double power = 0.6;

    // Claw
    private Servo Claw;
    private boolean clawIsOpen = false;
    private boolean buttonClawIsPressed = false;


    // SlowMode
    private boolean slowModeOn = false;
    private boolean buttonSlowIsPressed = false;


    @Override
    public void init() {

        //Add Drive Mode To Telemetry
        telemetry.addData("Drive Mode","Slow");

        // Initialize Wheels
        telemetry.addData("I", "Initializing Wheels");
        telemetry.update();

        WheelFrontLeft = hardwareMap.dcMotor.get("WheelFL");
        WheelFrontRight = hardwareMap.dcMotor.get("WheelFR");
        WheelBackLeft = hardwareMap.dcMotor.get("WheelBL");
        WheelBackRight = hardwareMap.dcMotor.get("WheelBR");

        Lift = hardwareMap.dcMotor.get("LiftW");

        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift.setDirection(DcMotorSimple.Direction.FORWARD);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Claw
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setDirection(Servo.Direction.FORWARD);
        CloseClaw();

        // Let the user know initialization is complete.
        telemetry.addData("I", "Initialization Complete!");
        telemetry.update();

    }

    @Override
    public void loop() {

        /*
         * Gamepad Controls
         */

        double oneLeftStickYPower = -gamepad1.left_stick_y;
        double oneLeftStickXPower = gamepad1.left_stick_x;
        double oneRightStickXPower = gamepad1.right_stick_x;
        boolean oneButtonA = gamepad1.a;
        boolean twoButtonA = gamepad2.a;
        boolean twoUpPad = gamepad2.dpad_up;
        boolean twoDownPad = gamepad2.dpad_down;


        /*
         * Do Stuff Here!
         */

        // Drive Controls
        ProMotorControl(oneLeftStickYPower, oneLeftStickXPower, oneRightStickXPower);

        // Slow Controls
        ToggleSlowMode(oneButtonA);

        // Claw Controls
        ToggleClaw(twoButtonA);

        // Lift
        setLift(twoUpPad, twoDownPad);

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
        double powerLeftY = -left_stick_y;  // DRIVE : Backward -1 <---> 1 Forward
        double powerLeftX = left_stick_x*-1;   // STRAFE:     Left -1 <---> 1 Right
        double powerRightX = -right_stick_x; // ROTATE:     Left -1 <---> 1 Right

        double r = Math.hypot(powerLeftX, powerLeftY);
        double robotAngle = Math.atan2(powerLeftY, powerLeftX) - Math.PI / 4;
        double leftX = powerRightX;
        final double v1 = r * Math.cos(robotAngle) + leftX;
        final double v2 = r * Math.sin(robotAngle) - leftX;
        final double v3 = r * Math.sin(robotAngle) + leftX;
        final double v4 = r * Math.cos(robotAngle) - leftX;

        telemetry.addData("Magnus Is Ready","");


        telemetry.addData("Wheel Front Left",v1*power);
        telemetry.addData("Wheel Front Right",v2*power);
        telemetry.addData("Wheel Back Left",v3*power);
        telemetry.addData("Wheel Back Right",v4*power);

        WheelFrontLeft.setPower(v1*power);
        WheelFrontRight.setPower(v2*power);
        WheelBackLeft.setPower(v3*power);
        WheelBackRight.setPower(v4*power);

    }

    private void ToggleSlowMode(boolean button) {
        if (button && !buttonSlowIsPressed) {
            buttonSlowIsPressed = true;
            if (slowModeOn) {
                power = 0.9;
                telemetry.addData("Drive Mode","Fast");
            } else {
                power = 0.6;
                telemetry.addData("Drive Mode","Slow");
            }
            slowModeOn = !slowModeOn;
        }

        if (!button) {
            buttonSlowIsPressed = false;
        }

        if (slowModeOn) {
            telemetry.addData("Drive Mode","Fast");
        } else {
            telemetry.addData("Drive Mode","Slow");
        }
    }

    private void setLift(boolean upPad,boolean downPad) {
        if(upPad) {
            Lift.setPower(0.75);
        }
        if(downPad){
            Lift.setPower(-0.75);
        }
        if(!upPad && !downPad){
            Lift.setPower(0);
        }
    }

    private void ToggleClaw(boolean button) {
        if (button && !buttonClawIsPressed) {
            buttonClawIsPressed = true;
            if (clawIsOpen) {
                OpenClaw();
            } else {
                CloseClaw();
            }
            clawIsOpen = !clawIsOpen;
        }

        if (!button) {
            buttonClawIsPressed = false;
        }

        if (clawIsOpen) {
            telemetry.addData("CLAW","Open");
        } else {
            telemetry.addData("CLAW","Close");
        }

        telemetry.addData("Claw Position", Claw.getPosition());
    }

    private void CloseClaw() {
        Claw.setPosition(1);
    }

    private void OpenClaw() {
        Claw.setPosition(0.3);
    }
}
