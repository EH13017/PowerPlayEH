package org.firstinspires.ftc.teamcode.Comp.Programs.NoTurret;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


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


   double power = 0.6;

//   // Claw
//   private Servo Claw;
//   private boolean clawIsOpen = true;
//   private boolean buttonClawIsPressed = false;
//   private final double CLAW_CLOSE = 0.3;
//   private final double CLAW_OPEN = 0.75;

   // SlowMode
   private boolean slowModeOn = false;
   private boolean buttonSlowIsPressed = false;

//   // REV Blinkin
//   private RevBlinkinLedDriver LED;


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
//      Claw = hardwareMap.get(Servo.class, "Claw");
//      Claw.setDirection(Servo.Direction.FORWARD);
//      OpenClaw();
//
//      // REV Blinkin Initialization
//      LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
//      setLEDPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

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
      float twoTrigerLeft = gamepad2.left_trigger;
      float twoTrigerRight = gamepad2.right_trigger;
      boolean twoUpPad = gamepad2.dpad_up;
      boolean twoDownPad = gamepad2.dpad_down;
      boolean twoLeftPad = gamepad2.dpad_left;
      boolean twoRightPad = gamepad2.dpad_right;
      boolean twoBack = gamepad2.back;
      boolean twoBumperLeft = gamepad2.left_bumper;
      boolean twoBumperRight = gamepad2.right_bumper;
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

      // Lift
      setLift(twoUpPad, twoDownPad);

      getLiftTelemetry();

      // Auto Claw TODO: Set lift values
//      AutoClaw(twoBack,
//               twoButtonA,
//               twoButtonB,
//               twoButtonX,
//               twoButtonY);

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
            power = 0.4;
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

   private void autoLift(int height) {
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

//   private void ToggleClaw(boolean button) {
//      if (button && !buttonClawIsPressed) {
//         buttonClawIsPressed = true;
//         if (clawIsOpen) {
//            CloseClaw();
//         } else {
//            OpenClaw();
//         }
//         clawIsOpen = !clawIsOpen;
//      }
//
//      if (!button) {
//         buttonClawIsPressed = false;
//      }
//
//      if (clawIsOpen) {
//         telemetry.addData("CLAW","Open");
//      } else {
//         telemetry.addData("CLAW","Close");
//      }
//
//      telemetry.addData("Claw Position", Claw.getPosition());
//   }

   private void AutoClaw(
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

      if (heightLift != -1) {
         do {
            autoLift(heightLift);
            getLiftTelemetry();
         } while ((LiftLeft.isBusy() || LiftRight.isBusy()) && !backButton);

         //CODE LIGHTS Black
         heightLift = -1;
      }

      telemetry.update();
   }

   private void getLiftTelemetry() {
      telemetry.addData("LiftL Encoder", LiftLeft.getCurrentPosition());
      telemetry.addData("LiftR Encoder", LiftRight.getCurrentPosition());
   }




//   private void CloseClaw() {
//      Claw.setPosition(CLAW_CLOSE);
//   }
//
//   private void OpenClaw() {
//      Claw.setPosition(CLAW_OPEN);
//   }
//
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
