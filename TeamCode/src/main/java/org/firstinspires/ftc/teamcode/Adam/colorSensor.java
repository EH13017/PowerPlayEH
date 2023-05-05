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


@Autonomous(name = "Color Sensor Find", group = "Testing")
public class colorSensor extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;

    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        WheelFrontLeft = hardwareMap.dcMotor.get("WheelFL");
        WheelFrontRight = hardwareMap.dcMotor.get("WheelFR");
        WheelBackLeft = hardwareMap.dcMotor.get("WheelBL");
        WheelBackRight = hardwareMap.dcMotor.get("WheelBR");

        color.enableLed(true);


        int colorValue;
        colorValue = 0;


        int colorToGo = 0;


        if(color.red()>=color.green() & color.red()>=color.blue()){
            telemetry.addData("Color","Red");
            colorToGo = 1;
        }
        if(color.green()>=color.red() & color.green()>=color.blue()){
            telemetry.addData("Color","Green");
            colorToGo = 2;
        }
        if(color.blue()>=color.red() & color.blue()>=color.green()){
            telemetry.addData("Color","Blue");
            colorToGo = 3;
        }



        telemetry.addData("Color To Go", colorToGo);
        telemetry.update();



        // Wait for the Play button to be pressed
        waitForStart();


        telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());
        telemetry.update();



        while (colorValue != colorToGo){
            colorValue = 0;

            if(color.red()>=color.green() & color.red()>=color.blue()){
                telemetry.addData("Color","Red");
                colorValue = 1;
            }
            if(color.green()>=color.red() & color.green()>=color.blue()){
                telemetry.addData("Color","Green");
                colorValue = 2;
            }
            if(color.blue()>=color.red() & color.blue()>=color.green()){
                telemetry.addData("Color","Blue");
                colorValue = 3;
            }



            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());

            telemetry.update();

            WheelFrontLeft.setPower(0.25);
            WheelFrontRight.setPower(-0.25);
            WheelBackLeft.setPower(0.25);
            WheelBackRight.setPower(-0.25);
        }

        WheelFrontLeft.setPower(0);
        WheelFrontRight.setPower(0);
        WheelBackLeft.setPower(0);
        WheelBackRight.setPower(0);


        if(color.red()>=color.green() & color.red()>=color.blue()){
            telemetry.addData("Color","Red");
            colorValue = 1;
        }
        if(color.green()>=color.red() & color.green()>=color.blue()){
            telemetry.addData("Color","Green");
            colorValue = 2;
        }
        if(color.blue()>=color.red() & color.blue()>=color.green()){
            telemetry.addData("Color","Blue");
            colorValue = 3;
        }



        telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());

        telemetry.update();



        sleep(500);



        if(color.red()>=color.green() & color.red()>=color.blue()){
            telemetry.addData("Color","Red");
            colorValue = 1;
        }
        if(color.green()>=color.red() & color.green()>=color.blue()){
            telemetry.addData("Color","Green");
            colorValue = 2;
        }
        if(color.blue()>=color.red() & color.blue()>=color.green()){
            telemetry.addData("Color","Blue");
            colorValue = 3;
        }



        telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());

        telemetry.update();



        while (colorValue != colorToGo){



            if(color.red()>=color.green() & color.red()>=color.blue()){
                telemetry.addData("Color","Red");
                colorValue = 1;
            }
            if(color.green()>=color.red() & color.green()>=color.blue()){
                telemetry.addData("Color","Green");
                colorValue = 2;
            }
            if(color.blue()>=color.red() & color.blue()>=color.green()){
                telemetry.addData("Color","Blue");
                colorValue = 3;
            }



            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());

            telemetry.update();



            WheelFrontLeft.setPower(-0.1);
            WheelFrontRight.setPower(0.1);
            WheelBackLeft.setPower(-0.1);
            WheelBackRight.setPower(0.1);
        }

        WheelFrontLeft.setPower(0);
        WheelFrontRight.setPower(0);
        WheelBackLeft.setPower(0);
        WheelBackRight.setPower(0);

    }
}