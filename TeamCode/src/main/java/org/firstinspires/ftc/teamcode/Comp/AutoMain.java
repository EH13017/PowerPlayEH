package org.firstinspires.ftc.teamcode.Comp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Interfaces.IVision;

import java.util.List;

@Autonomous(name = "Auto Competition", group = "Competition")
public class AutoMain extends LinearOpMode {

   private DcMotor WheelFrontLeft;
   private DcMotor WheelFrontRight;
   private DcMotor WheelBackLeft;
   private DcMotor WheelBackRight;

   ColorSensor color;

   @Override
   public void runOpMode() throws InterruptedException {

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

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        color = hardwareMap.get(ColorSensor.class, "Color");

      waitForStart();

      while (opModeIsActive()) {      // If we've stopped the robot, stop the program

            ProMotorControl(0.05, 0.0, 0.0);  // Forward
            sleep(2600);
            ProMotorControl(0.0, 0.0, 0.0);  // Stop

           int colorValue;
          colorValue = 0;

          if(color.red()>color.green() & color.red()> color.blue()){
               telemetry.addData("Color","Red");
               colorValue = 1;
               //Left
           }
           if(color.green()>color.red() & color.green()> color.blue()){
               telemetry.addData("Color","Green");
               colorValue = 2;
               //Park
           }
           if(color.blue()>color.red() & color.blue()> color.green()){
               telemetry.addData("Color","Blue");
               colorValue = 3;
               //Right
           }

           //sleep(3000);

           //ProMotorControl(0.05, 0.0, 0.0);  // Forward
           //sleep(1250);
           //ProMotorControl(0.0, 0.0, 0.0);  // Stop

          telemetry.addData("Red Value:",color.red());
          telemetry.addData("Green Value:",color.green());
          telemetry.addData("Blue Value:",color.blue());

          telemetry.update();

           if(colorValue == 1){
               ProMotorControl(0.0,-0.1,0.0);
               sleep(2150);
               ProMotorControl(0.0, 0.0, 0.0);
           }

          if(colorValue == 2){
              ProMotorControl(-0.1,0.0,0.0);
              sleep(500);
              ProMotorControl(0.0, 0.0, 0.0);
          }

          if(colorValue == 3){
              ProMotorControl(0.0,0.1,0.0);
              sleep(2150);
              ProMotorControl(0.0, 0.0, 0.0);
          }



           telemetry.update();


            break; // End the program once it has finished
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

}
