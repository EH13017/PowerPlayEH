package org.firstinspires.ftc.teamcode.Adam;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Testing", group = "Testing")
public class ClawTesting extends OpMode {

    /*
     * Declare Hardware
     */


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
    private boolean clawIsOpen = true;
    private boolean buttonClawIsPressed = false;
    private final double CLAW_CLOSE = 0.3;
    private final double CLAW_OPEN = 0.75;

    // Grabber
    private Servo Grab;
    private boolean grabIsOpen = true;
    private boolean buttonGrabIsPressed = false;
    private final double GRAB_CLOSE = 0.2;
    private final double GRAB_OPEN = 0.5;



    @Override
    public void init() {


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
        OpenClaw();

        Grab = hardwareMap.get(Servo.class, "Grab");
        Grab.setDirection(Servo.Direction.FORWARD);
        OpenGrab();

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

        // Claw Controls
        ToggleClaw(twoBumperLeft);
        ToggleGrab(twoBumperRight);

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
        // Set the target position
        Lift.setTargetPosition(height);
        // Switch to RUN_TO_POSITION mode
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Get the motor moving by setting the max velocity in ticks per second
        Lift.setVelocity(MAX_LIFT_VELOCITY);

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
                CloseClaw();
            } else {
                OpenClaw();
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

    private void ToggleGrab(boolean button) {
        if (button && !buttonGrabIsPressed) {
            buttonGrabIsPressed = true;
            if (grabIsOpen) {
                CloseGrab();
            } else {
                OpenGrab();
            }
            grabIsOpen = !grabIsOpen;
        }

        if (!button) {
            buttonGrabIsPressed = false;
        }

        if (grabIsOpen) {
            telemetry.addData("GRAB","Open");
        } else {
            telemetry.addData("GRAB","Close");
        }

        telemetry.addData("Grab Position", Grab.getPosition());
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

//        OpenGrab();

        if(heightLift != -1 && directionRotate != -1) {
            if (heightLift == GROUND && (directionRotate == EAST || directionRotate == WEST)) {
                // Do nothing
            } else {

                // Retract scissor
                // Sleep for a bit
                CloseGrab();
                sleep(500);
                CloseClaw();
                sleep(500);

                do {
                    if (heightLift == GROUND && (directionRotate == NORTH || directionRotate == SOUTH)) {
                        autoLift(LOW); // Lift to Low
                    } else {
                        autoLift(heightLift); // Lift to height
                    }
                    getTurriftTelemetry();
                } while (Lift.isBusy() && !backButton);

                if (heightLift == GROUND && directionRotate == NORTH) {
                    CloseGrab();
                } else {
                    OpenGrab();
                }

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

                if (heightLift == GROUND && directionRotate == NORTH) {
                    OpenClaw();
                    sleep(500);
                    OpenGrab();
                    sleep(500);
                }

                if (heightLift != GROUND && directionRotate != NORTH) {
                    // Extend scissors
                    // Sleep for a bit
                }
            }

            //CODE LIGHTS Black
            heightLift = -1;
            directionRotate = -1;
            telemetry.addData("Position Lift", heightLift);
        }

        telemetry.update();
    }

    private void getTurriftTelemetry() {
        telemetry.addData("Lift Encoder", Turret.getCurrentPosition());
        telemetry.addData("Turret Encoder", Turret.getCurrentPosition());
    }




    private void CloseClaw() {
        Claw.setPosition(CLAW_CLOSE);
    }

    private void OpenClaw() {
        Claw.setPosition(CLAW_OPEN);
    }

    private void CloseGrab() {
        Grab.setPosition(GRAB_CLOSE);
    }

    private void OpenGrab() {
        Grab.setPosition(GRAB_OPEN);
    }
}
