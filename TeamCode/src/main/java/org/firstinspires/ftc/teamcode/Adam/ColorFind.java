package org.firstinspires.ftc.teamcode.Adam;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name = "Color Sensor Value Find", group = "Testing")
public class ColorFind extends LinearOpMode {
    ColorSensor color;
    @Override
    public void runOpMode() {
        int red = 0;
        int green = 0;
        int blue = 0;

        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        //Enable The LED
        color.enableLed(true);

        // Wait for the Play button to be pressed
        waitForStart();

        while (opModeIsActive()){
            if(color.red()>red){
                red = color.red();
            }
            if(color.blue()>blue){
                blue = color.blue();
            }
            if(color.green()>green){
                green = color.green();
            }

            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());

            telemetry.addData("Max Red", red);
            telemetry.addData("Max Green", green);
            telemetry.addData("Max Blue", blue);

            telemetry.update();
        }


    }
}
