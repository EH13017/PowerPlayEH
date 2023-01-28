package org.firstinspires.ftc.teamcode.Adam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    // Lift
    private DcMotorEx Lift;
    private int heightLift = -1;
    private final int ENCODER_COUNT_LIFT = 1120;
    private final int GROUND = 0;
    private final int LOW = -1000;
    private final int MEDIUM = -1750;
    private final int HIGH = -2400;
    private final double MAX_LIFT_SPEED = 0.75;
    private final int MAX_LIFT_VELOCITY = ENCODER_COUNT_LIFT*2;

    // Turret
    private DcMotorEx Turret;
    private int directionRotate = -1;
    private final int TURRET_ENCODER_COUNT = 288;
    private final int NORTH = 0;
    private final int EAST = TURRET_ENCODER_COUNT/4;
    private final int SOUTH = TURRET_ENCODER_COUNT/2;
    private final int WEST = -TURRET_ENCODER_COUNT/4;



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

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Lift
        Lift = hardwareMap.get(DcMotorEx.class, "LiftW");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Turret
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setDirection(DcMotorSimple.Direction.FORWARD);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        boolean twoButtonB = gamepad2.b;
        boolean twoButtonX = gamepad2.x;
        boolean twoButtonY = gamepad2.y;
        boolean twoUpPad = gamepad2.dpad_up;
        boolean twoDownPad = gamepad2.dpad_down;
        boolean twoLeftPad = gamepad2.dpad_left;
        boolean twoRightPad = gamepad2.dpad_right;
        boolean twoBack = gamepad2.back;
        boolean twoBumperLeft = gamepad2.left_bumper;
        boolean twoBumperRight = gamepad2.right_bumper;

        /*
         * Do Stuff Here!
         */

        // Drive Controls
        ProMotorControl(oneLeftStickYPower, oneLeftStickXPower, oneRightStickXPower);

        // Slow Controls
        ToggleSlowMode(oneButtonA);

        // Claw Controls
        ToggleClaw(twoBumperLeft);

        // Lift
//        setLift(twoBumperLeft, twoBumperRight);

        //Auto Claw
        AutoClaw(twoBack,
                twoUpPad,
                twoDownPad,
                twoLeftPad,
                twoRightPad,
                twoButtonA,
                twoButtonB,
                twoButtonX,
                twoButtonY);

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

    private void setLift(boolean up,boolean down) {
        if (up) {
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Lift.setPower(MAX_LIFT_SPEED);
            Lift.setVelocity(MAX_LIFT_VELOCITY);
        }
        if (down) {
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Lift.setPower(-MAX_LIFT_SPEED);
            Lift.setVelocity(-MAX_LIFT_VELOCITY);
        }
        if (!up && !down) {
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Lift.setPower(0);
            Lift.setVelocity(0);
        }
    }

    private void autoLift(int height) {
//        // If we have pressed a button to set the position...
//        if (setToPositionMode) {
            // Set the target position
            Lift.setTargetPosition(height);
            // Switch to RUN_TO_POSITION mode
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Get the motor moving by setting the max velocity in ticks per second
            Lift.setVelocity(MAX_LIFT_VELOCITY);
//        }

//        // Once the motor is finished moving to the position we have set...
//        if (!Lift.isBusy() && setToPositionMode) {
//            setToPositionMode = false;
//            LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }

    }

//    private void ProLiftControl(boolean levelZeroButtonPressed,
//                                boolean levelOneButtonPressed,
//                                boolean levelTwoButtonPressed,
//                                boolean levelThreeButtonPressed) {
//
//        // Set the target position based on which button is pressed TODO: Switch statements?
//
//        if (levelZeroButtonPressed) {
//            targetPos = LEVEL_ZERO;
//            BucketIn();
//            setToPositionMode = true;
//        } else if (levelOneButtonPressed) {
//            targetPos = LEVEL_ONE;
//            setToPositionMode = true;
//        } else if (levelTwoButtonPressed) {
//            targetPos = LEVEL_TWO;
//            setToPositionMode = true;
//        } else if (levelThreeButtonPressed) {
//            targetPos = LEVEL_THREE;
//            setToPositionMode = true;
//        }
//
//        // If we have pressed a button to set the position...
//        if (setToPositionMode) {
//            // Set the target position
//            LiftMotor.setTargetPosition(targetPos);
//            // Switch to RUN_TO_POSITION mode
//            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // Get the motor moving by setting the max velocity in ticks per second
//            LiftMotor.setVelocity(MAX_LIFT_VELOCITY);
//        }
//
//        // Once the motor is finished moving to the position we have set...
//        if (!LiftMotor.isBusy() && setToPositionMode) {
//            setToPositionMode = false;
//            LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//    }

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
            //telemetry.addData("CLAW","Open");
        } else {
            //telemetry.addData("CLAW","Close");
        }

        telemetry.addData("Claw Position", Claw.getPosition());
    }

    private void RotateTurret(int position) {
        Turret.setTargetPosition(position);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setVelocity(TURRET_ENCODER_COUNT*2);
    }

    private void AutoClaw(
                          boolean backButton,
                          boolean up,
                          boolean down,
                          boolean left,
                          boolean right,

                          boolean A,
                          boolean B,
                          boolean X,
                          boolean Y
    ){
        // Height
        if(A){
            heightLift = GROUND; //Green
        }
        if(B){
            heightLift = LOW; //Red
        }
        if(X){
            heightLift = MEDIUM; //Blue
        }
        if(Y){
            heightLift = HIGH; //Yellow
        }
        if(backButton && heightLift != -1){
            heightLift = -1; //Black
        }

        // Direction
        if(up && heightLift != -1){
            directionRotate = NORTH;//North
        }
        if(right && heightLift != -1){
            directionRotate = EAST;//East
        }
        if(down && heightLift != -1){
            directionRotate = SOUTH;//South
        }
        if(left && heightLift != -1){
            directionRotate = WEST;//West
        }

        telemetry.addData("Direction Rotate",directionRotate);

        if(heightLift != -1 && directionRotate != -1) {
            CloseClaw();

            do {
                if (heightLift == GROUND && (directionRotate == NORTH || directionRotate == SOUTH)) {
                    autoLift(LOW); // Lift to Low
                } else {
                    autoLift(heightLift); // Lift to height
                }
                getTurriftTelemetry();
            } while (Lift.isBusy() && !backButton);

            do {
                if (heightLift == GROUND && (directionRotate != NORTH && directionRotate != SOUTH)) {
                    // Do Nothing
                } else {
                    RotateTurret(directionRotate);
                }
                getTurriftTelemetry();
            } while (Turret.isBusy() && !backButton);

            Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Turret.setPower(0);

            do {
                if (heightLift == GROUND && (directionRotate == NORTH || directionRotate == SOUTH)) {
                    autoLift(GROUND); // Lift to Ground
                }
                getTurriftTelemetry();
            } while (Lift.isBusy() && !backButton);

            //CODE LIGHTS Black

            heightLift = -1;
            directionRotate = -1;
        }
        telemetry.addData("Position Lift", heightLift);


        telemetry.update();
    }

    private void getTurriftTelemetry() {
        telemetry.addData("Lift Encoder", Turret.getCurrentPosition());
        telemetry.addData("Turret Encoder", Turret.getCurrentPosition());
    }




    private void CloseClaw() {
        Claw.setPosition(1);
    }

    private void OpenClaw() {
        Claw.setPosition(0.3);
    }
}
