package org.firstinspires.ftc.teamcode.Comp.Programs.NoTurret;

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

   // Lift TODO: Set lift values
   private DcMotorEx LiftLeft;
   private DcMotorEx LiftRight;
   private int heightLift = -1;
   private final int ENCODER_COUNT_LIFT = 1120;
   private final int GROUND = 0;
   private final int LOW = -1750; // -1150;
   private final int MEDIUM = -2800; // -1800;
   private final int HIGH = -3850; // -2500;
   private final double MAX_LIFT_SPEED = 0.75;
   private final int MAX_LIFT_VELOCITY = ENCODER_COUNT_LIFT;

//   // Claw TODO: Set claw values
//   private Servo Claw;
//   private boolean clawIsOpen = true;
//   private boolean buttonClawIsPressed = false;
//   private final double CLAW_CLOSED = 0.3;
//   private final double CLAW_OPEN = 0.75;

   // SlowMode
   private boolean slowModeOn = true;
   private boolean buttonSlowIsPressed = false;
   private final double SLOW = 0.4;
   private final double FAST = 0.9;
   private double percentToSlow = SLOW;

//   // REV Blinkin
//   private RevBlinkinLedDriver LED;


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

      WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
      LiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
      LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//      // Initialize Claw
//      telemetry.addData("I", "Initializing Claw");
//      telemetry.update();
//
//      Claw = hardwareMap.get(Servo.class, "Claw");
//      Claw.setDirection(Servo.Direction.FORWARD);
//      OpenClaw(); // TODO: Uncomment this


//      // Initialize REV Blinkin
//      telemetry.addData("I", "Initializing Blinkin");
//      telemetry.update();
//
//      LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
//      setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


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

      // Gamepad 2
      boolean twoButtonA = gamepad2.a;
      boolean twoButtonB = gamepad2.b;
      boolean twoButtonX = gamepad2.x;
      boolean twoButtonY = gamepad2.y;
      float twoTriggerLeft = gamepad2.left_trigger;
      float twoTriggerRight = gamepad2.right_trigger;
      boolean twoPadUp = gamepad2.dpad_up;
      boolean twoPadDown = gamepad2.dpad_down;
      boolean twoPadLeft = gamepad2.dpad_left;
      boolean twoPadRight = gamepad2.dpad_right;
      boolean twoBumperLeft = gamepad2.left_bumper;
      boolean twoBumperRight = gamepad2.right_bumper;
      boolean twoBack = gamepad2.back;
      boolean twoStart = gamepad2.start;

      /*
       * Do Stuff Here!
       */

//      // LEDs
//      manageLEDColors();

      // Drive Controls
      ProMotorControl(oneLeftStickYPower, oneLeftStickXPower, oneRightStickXPower);

      // Slow Controls
      ToggleSlowMode(oneButtonA);

      // Lift Controls
      setLift(twoPadUp, twoPadDown);
      GetLiftTelemetry();

//      // Auto Lift Controls TODO: Set lift values
//      AutoLift(twoBack,
//               twoButtonA,
//               twoButtonB,
//               twoButtonX,
//               twoButtonY);

//      // Claw Controls TODO: Set claw values
//      ToggleClaw(twoBumperRight);

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

      telemetry.addData("Wheel Front Left",v1* percentToSlow);
      telemetry.addData("Wheel Front Right",v2* percentToSlow);
      telemetry.addData("Wheel Back Left",v3* percentToSlow);
      telemetry.addData("Wheel Back Right",v4* percentToSlow);

      WheelFrontLeft.setPower(v1* percentToSlow);
      WheelFrontRight.setPower(v2* percentToSlow);
      WheelBackLeft.setPower(v3* percentToSlow);
      WheelBackRight.setPower(v4* percentToSlow);

   }

   private void ToggleSlowMode(boolean button) {
      if (button && !buttonSlowIsPressed) {
         buttonSlowIsPressed = true;
         slowModeOn = !slowModeOn;
      }

      if (!button) {
         buttonSlowIsPressed = false;
      }

      if (slowModeOn) {
         percentToSlow = SLOW;
         telemetry.addData("Drive Mode","Slow: " + SLOW + "% Power");
      } else {
         percentToSlow = FAST;
         telemetry.addData("Drive Mode","Fast: " + FAST + "% Power");
      }
   }

   private void setLift(boolean up,boolean down) {

      LiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      LiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      if (up) {
//         LiftLeft.setPower(MAX_LIFT_SPEED);
//         LiftRight.setPower(MAX_LIFT_SPEED);
         LiftLeft.setVelocity(-MAX_LIFT_VELOCITY);
         LiftRight.setVelocity(-MAX_LIFT_VELOCITY);
      }
      else if (down) {
//         LiftLeft.setPower(-MAX_LIFT_SPEED);
//         LiftRight.setPower(-MAX_LIFT_SPEED);
         LiftLeft.setVelocity(MAX_LIFT_VELOCITY);
         LiftRight.setVelocity(MAX_LIFT_VELOCITY);
      }
      else {
//         LiftLeft.setPower(0);
//         LiftRight.setPower(0);
         LiftLeft.setVelocity(0);
         LiftRight.setVelocity(0);
      }
   }

   private void SetLift(int height) {
      // Set the target position
      LiftLeft.setTargetPosition(height);
      LiftRight.setTargetPosition(height);
      // Switch to RUN_TO_POSITION mode
      LiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // Get the motor moving by setting the max velocity in ticks per second
      LiftLeft.setVelocity(MAX_LIFT_VELOCITY);
      LiftRight.setVelocity(MAX_LIFT_VELOCITY);

   }

   private void AutoLift(
           boolean backButton,
           boolean A,
           boolean B,
           boolean X,
           boolean Y
   ){
      // Height
      if      (A) { heightLift = GROUND; } // Green
      else if (B) { heightLift = LOW;    } // Red
      else if (X) { heightLift = MEDIUM; } // Blue
      else if (Y) { heightLift = HIGH;   } // Yellow
      else if (backButton && heightLift != -1) {
         heightLift = -1; // Black
      }

      // Move to Height
      if (heightLift != -1) {
         do {
            SetLift(heightLift);
            GetLiftTelemetry();
         } while ((LiftLeft.isBusy() || LiftRight.isBusy()) && !backButton);

         //CODE LIGHTS Black
         heightLift = -1;
      }

      telemetry.update();
   }

   private void GetLiftTelemetry() {
      telemetry.addData("LiftL Encoder", LiftLeft.getCurrentPosition());
      telemetry.addData("LiftR Encoder", LiftRight.getCurrentPosition());
   }

//   private void ToggleClaw(boolean button) {
//      if (button && !buttonClawIsPressed) {
//         buttonClawIsPressed = true;
//         clawIsOpen = !clawIsOpen;
//      }
//
//      if (!button) {
//         buttonClawIsPressed = false;
//      }
//
//      if (clawIsOpen) {
//         OpenClaw();
//         telemetry.addData("Claw Position","Open: " + CLAW_OPEN);
//      } else {
//         CloseClaw();
//         telemetry.addData("Claw Position","Closed: " + CLAW_CLOSED);
//      }
//   }
//
//   private void CloseClaw() { Claw.setPosition(CLAW_CLOSED); }
//
//   private void OpenClaw() { Claw.setPosition(CLAW_OPEN); }

//   // Here is a file to show how to use the REV Blinkin, along with a complete list of colors:
//   // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
//   protected void setLEDPattern(RevBlinkinLedDriver.BlinkinPattern setPattern) {
//      LED.setPattern(setPattern);
//   }
//
//   protected void turnOffLEDPattern() {
//      LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//   }
//
//   private void manageLEDColors() {
//      if (heightLift == GROUND) {
//         setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//      }
//      else if (heightLift == LOW) {
//         setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//      }
//      else if (heightLift == MEDIUM) {
//         setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//      }
//      else if (heightLift == HIGH) {
//         setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//      }
//      else {
//         turnOffLEDPattern();
//      }
//   }
}
