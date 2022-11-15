package org.firstinspires.ftc.teamcode.Lessons;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Touch Sensors - Completed", group = "Lessons")
public class Lesson02_TouchSensorsCompleted extends OpMode {


    /*
     * Declare Hardware
     */

    // Touch Sensor
    private TouchSensor Touch;
    private boolean isPressed = false;

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
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        LED.setPattern(pattern);

        telemetry.addData("Initialization", "Complete!");
        telemetry.update();

    }

    @Override
    public void loop() {

        /*
         * Do stuff here!
         */

        isPressed = Touch.isPressed();

        if (isPressed) {
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            LED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        telemetry.addData("Sensor Data", isPressed);

        telemetry.update();
    }

}
