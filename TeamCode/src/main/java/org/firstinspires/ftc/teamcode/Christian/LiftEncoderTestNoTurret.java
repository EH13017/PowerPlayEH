package org.firstinspires.ftc.teamcode.Christian;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lift Encoder Test", group = "Testing")
public class LiftEncoderTestNoTurret extends OpMode {

    /*
     * Declare Hardware
     */

    // Lift TODO: Set lift values
    private DcMotorEx LiftLeft;
    private DcMotorEx LiftRight;
    private double heightLiftLeft = -1;
    private double heightLiftRight = -1;
    private final double ENCODER_COUNT_LIFT_60 = 1680; // 60:1, Left
    private final double ENCODER_COUNT_LIFT_40 = -1120; // 40:1, Right
    private final double ROTATIONS_GROUND = 0;
    private final double ROTATIONS_LOW = 1.5; // -1680
    private final double ROTATIONS_MEDIUM = 2.5; // -2800
    private final double ROTATIONS_HIGH = 3.5; // -3850
    private final double MAX_LIFT_SPEED = 0.75;
    private final double MAX_LIFT_VELOCITY_LEFT = ENCODER_COUNT_LIFT_60*2; // 60:1, Left
    private final double MAX_LIFT_VELOCITY_RIGHT = ENCODER_COUNT_LIFT_40*2; // 40:1, Right


    @Override
    public void init() {

        // Initialize Lift
        telemetry.addData("I", "Initializing Lift");
        telemetry.update();

        LiftLeft = hardwareMap.get(DcMotorEx.class, "LiftL");
        LiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftRight = hardwareMap.get(DcMotorEx.class, "LiftR");
        LiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Let the user know initialization is complete.
        telemetry.addData("I", "Initialization Complete! :D");
        telemetry.update();

    }

    @Override
    public void loop() {

        /*
         * Gamepad Controls
         */

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

        // Lift
//        LiftManual(twoUpPad, twoDownPad);
        LiftAuto(twoBack,
                 twoButtonA,
                 twoButtonB,
                 twoButtonX,
                 twoButtonY,
                 twoUpPad,
                 twoDownPad);

        telemetry.update();

    }


    /*
     * Methods
     */

    private void LiftManual(boolean up, boolean down) {
        if (up) {
//            LiftLeft.setPower(MAX_LIFT_SPEED);
//            LiftRight.setPower(-MAX_LIFT_SPEED);
            LiftLeft.setVelocity(MAX_LIFT_VELOCITY_LEFT);
            LiftRight.setVelocity(MAX_LIFT_VELOCITY_RIGHT);
        }
        else if (down) {
//            LiftLeft.setPower(-MAX_LIFT_SPEED);
//            LiftRight.setPower(MAX_LIFT_SPEED);
            LiftLeft.setVelocity(-MAX_LIFT_VELOCITY_LEFT);
            LiftRight.setVelocity(-MAX_LIFT_VELOCITY_RIGHT);
        }
        else {
//            LiftLeft.setPower(0);
//            LiftRight.setPower(0);
            LiftLeft.setVelocity(0);
            LiftRight.setVelocity(0);
        }
    }

    private void LiftMove(double heightLeft, double heightRight) {
        // Set the target position
        LiftLeft.setTargetPosition((int)heightLeft);
        LiftRight.setTargetPosition((int)heightRight);
        // Switch to RUN_TO_POSITION mode
        LiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Get the motor moving by setting the max velocity in ticks per second
        LiftLeft.setVelocity(MAX_LIFT_VELOCITY_LEFT);
        LiftRight.setVelocity(MAX_LIFT_VELOCITY_RIGHT);

        GetLiftTelemetry();
        telemetry.update();
    }

    private void SetLiftHeights(double rotations) {
        heightLiftLeft = ENCODER_COUNT_LIFT_60 * rotations;
        heightLiftRight = ENCODER_COUNT_LIFT_40 * rotations;
    }

    private void LiftAuto(
            boolean backButton,
            boolean A,
            boolean B,
            boolean X,
            boolean Y,
            boolean up,
            boolean down
    ) {
        // Height
        if      (A) { SetLiftHeights(ROTATIONS_GROUND); } // Green
        else if (B) { SetLiftHeights(ROTATIONS_LOW);    } // Red
        else if (X) { SetLiftHeights(ROTATIONS_MEDIUM); } // Blue
        else if (Y) { SetLiftHeights(ROTATIONS_HIGH);   } // Yellow
        else if (backButton && (heightLiftLeft != -1 && heightLiftRight != -1)) {
            heightLiftLeft = -1; // Black
            heightLiftRight = -1; // Black
        }

        // Move to Height
        if (heightLiftLeft != -1 && heightLiftRight != -1) {
            do {
                LiftMove(heightLiftLeft, heightLiftRight);
            } while ((LiftLeft.isBusy() || LiftRight.isBusy()) && !backButton && (!up && !down));

            LiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // CODE LIGHTS Black
            heightLiftLeft = -1;
            heightLiftRight = -1;
        }

        LiftManual(up, down);

        telemetry.update();
    }

    private void GetLiftTelemetry() {
        telemetry.addData("LiftL Encoder", LiftLeft.getCurrentPosition());
        telemetry.addData("LiftR Encoder", LiftRight.getCurrentPosition());
    }
}