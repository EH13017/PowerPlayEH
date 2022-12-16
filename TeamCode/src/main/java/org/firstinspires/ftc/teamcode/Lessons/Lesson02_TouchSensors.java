package org.firstinspires.ftc.teamcode.Lessons;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp(name = "Touch Sensors", group = "Lessons")
public class Lesson02_TouchSensors extends OpMode {


    /*
     * Declare Hardware
     */

    // Touch Sensor
    private TouchSensor Touch;
    private boolean isPressed = false;

    private boolean toggle = false;
    private boolean firstTime = true;

    // REV Blinkin
    private RevBlinkinLedDriver LED;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    @Override
    public void init() {

        /*
         * Initialization
         */

        telemetry.addData("Initialization", "Incomplete");
        telemetry.update();

        // Initialize Touch Sensor
        Touch = hardwareMap.get(TouchSensor.class, "Touch");

        // REV Blinkin Initialization
        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        LED.setPattern(pattern);

        telemetry.addData("Initialization", "Complete!");
        telemetry.update();

    }

    @Override
    public void loop() {

        /////////////////////////////////////////////////////////////////////////////////////////
        /**
         * This lesson will teach you the basics of how touch sensors and LED's work, along with
         * some practice in logic. So without further ado, let's jump straight into it!
         *
         * Your goal is to make the lights on the robot turn on when a touch sensor is pressed,
         * and turn off when it is not pressed. A few tips to help you get started:
         *
         * To tell whether the touch sensor is pressed, use this:
         Touch.isPressed()
         *
         * To set the colors of the LED, use this:
         LED.setPattern([YOUR COLOR])
         * The color you choose *must* be in all caps, otherwise it won't work.
         *
         * When you start to type in which color you want to use, a list of colors similar to what
         * you typed will appear. This is called autocomplete. You can use the arrow keys to choose
         * which autocomplete you want, then press the TAB key to select it.
         */
        /////////////////////////////////////////////////////////////////////////////////////////

        /*
         * Do stuff here!
         */
//        if (Touch.isPressed()) {
//            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//        }
//else{
//    LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//        }


        if (Touch.isPressed())  {
            if (toggle&&firstTime){
                LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                toggle = false;
                firstTime = false;
            }

            else if(!toggle&&firstTime){
                LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                toggle = true;
                firstTime = false;
            }
        }
        else if(!Touch.isPressed() &&!firstTime){
            firstTime = true;
        }
        telemetry.update();

    }
}
