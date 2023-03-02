package org.firstinspires.ftc.teamcode.Christian;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Testing", group = "Testing")
public class ClawTestingNoTurret extends OpMode {

    /*
     * Declare Hardware
     */

    // Claw
    private Servo Claw;
    private boolean clawIsOpen = true;
    private boolean buttonClawIsPressed = false;
    private double clawPosition = 0.0;
    private boolean buttonIncrementIsPressed = false;
    private boolean buttonDecrementIsPressed = false;
    private final double CLAW_OPEN = 0.0;
    private final double CLAW_CLOSED = 0.55;



    @Override
    public void init() {

        // Initialize Claw
        telemetry.addData("I", "Initializing Claw");
        telemetry.update();

        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setDirection(Servo.Direction.FORWARD);
        ZeroClaw();

        // Let the user know initialization is complete.
        telemetry.addData("I", "Initialization Complete!");
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

        // Gamepad 2
        boolean twoButtonA = gamepad2.a;
        boolean twoButtonB = gamepad2.b;
        boolean twoButtonX = gamepad2.x;
        boolean twoButtonY = gamepad2.y;
        boolean twoPadUp = gamepad2.dpad_up;
        boolean twoPadDown = gamepad2.dpad_down;
        boolean twoPadLeft = gamepad2.dpad_left;
        boolean twoPadRight = gamepad2.dpad_right;
        float twoTriggerLeft = gamepad2.left_trigger;
        float twoTriggerRight = gamepad2.right_trigger;
        boolean twoBumperLeft = gamepad2.left_bumper;
        boolean twoBumperRight = gamepad2.right_bumper;
        boolean twoBack = gamepad2.back;
        boolean twoStart = gamepad2.start;

        /*
         * Do Stuff Here!
         */

        // Claw Controls
        ToggleClaw(twoBumperRight);
        IncrementClawPosition(twoPadUp, twoPadDown);

        telemetry.update();

    }


    /*
     * Methods
     */


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
   }

   private void IncrementClawPosition(boolean buttonInc, boolean buttonDec) {
       if (buttonInc && !buttonIncrementIsPressed && clawPosition <= 1.0) {
           buttonIncrementIsPressed = true;
           clawPosition += 0.05;
       } else if (clawPosition > 1.0) { clawPosition = 1.0; }
       if (!buttonInc) { buttonIncrementIsPressed = false; }

       if (buttonDec && !buttonDecrementIsPressed && clawPosition >= 0.0) {
           buttonDecrementIsPressed = true;
           clawPosition -= 0.05;
       } else if (clawPosition < 0.0) { clawPosition = 0.0; }
       if (!buttonDec) { buttonDecrementIsPressed = false; }

       Claw.setPosition(clawPosition);

       GetClawTelemetry();
   }

    private void ZeroClaw() { Claw.setPosition(0.0); }

    private void CloseClaw() { clawPosition = CLAW_CLOSED; }

    private void OpenClaw() { clawPosition = CLAW_OPEN; }

    private void GetClawTelemetry() { telemetry.addData("Claw Position", Claw.getPosition()); }
}