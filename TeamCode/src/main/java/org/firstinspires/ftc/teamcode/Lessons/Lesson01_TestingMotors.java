package org.firstinspires.ftc.teamcode.Lessons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Testing Motors", group = "Lessons")
public class Lesson01_TestingMotors extends LinearOpMode {

    /////////////////////////////////////////////////////////////////////////////////////////
    /**
     * So! Motors! The things that spin and make your robot move when you drive
     * it. We are going to learn  how to code them. There are several parts to
     * coding a motor, but once you have coded one, it will become pretty easy
     * to code more. So put your nerd glasses on and grab your rubber duck, and
     * let's start learning!
     *
     * > Coding Tip: Many programmers have a rubber duck that they use to help them
     * > fix their code. The trick is to explain how your code works to the duck,
     * > as if the duck knew nothing at all about coding. This way, you'll explain
     * > mentally go through every last detail of your code, and you'll often
     * > find bugs you had previously glossed over.
     *
     */
    /////////////////////////////////////////////////////////////////////////////////////////
    /**
     * ) Step 1: Declaring the Hardware
     * This step is fairly simple, but to understand what the code is doing, you
     * need to first learn what a variable is and how to code one.
     *
     * A variable is like a container that is able to store data. Variables can
     * hold many different types of data, such as words (called strings), numbers
     * (called either integers, doubles, or floats), or true/false values (called
     * booleans). When you code a variable into existence, it is called "declaring"
     * a variable.
     *
     * A variable consists of three required parts, and one optional part (a total of four):
     *  - The scope of the variable (what is allowed to see what's in it)
     *  - The variable type (what sort of value it holds)
     *  - The name of the variable
     *  - The value of the variable
     *
     * The format is as such:
     * > scope type name = value;
     * For example:
       // private double threePointFive = 3.5;
     *
     * You may also declare empty variables by leaving out its value. You can still
     * set the value again later by typing `name = value` (you don't need to include
     * the scope or type when changing a variable's value).
     *
     * At this point, it isn't important to know the deep details of how scope works.
     * For now, just know that most of the time you'll be using `private` as the scope,
     * unless I say otherwise.
     *
     * So! Now that you know what a variable is, I can explain how to declare our hardware.
     * In order to do this, we are going to need an empty variable with the special data type
     * `DcMotor`. This data type is specific to the motors that we use for our robot. You may
     * name the motor whatever you like, but make sure it makes some sort of sense, otherwise
     * you'll forget what your new motor is for. (I named mine "motor" for simplicity.)
     *
     * Code your new motor variable right underneath the "Declare Hardware" comment. Don't
     * forget to set the scope as `private`, and to put a semicolon at the end of the line.
     *
     */
    /////////////////////////////////////////////////////////////////////////////////////////
    /**
     * ) Step 2: Initializing the Motor
     * Great!! Now that you've declared your motor, we have to initialize it. Initializing
     * our hardware allows us to set the behaviour of it and decide how it acts. There are
     * five parts to initializing a motor. They might get a little complex, so I'll show
     * you all 5 parts and briefly explain what they do.
     *
         motor = hardwareMap.dcMotor.get("WheelFR");                  // This tells the code which motor port this variable is controlling.
         motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);       // This tells the motor to reset its built-in encoder, which is a device that tells us how far the motor has turned.
         motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);          // This tells the motor not to use the encoder, and to spin based on a power instead of a given encoder position.
         motor.setDirection(DcMotorSimple.Direction.FORWARD);         // This tells the motor to rotate clockwise when given a positive power, and counterclockwise when given a negative power.
         motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // This tells the motor whether or not to stop immediately (BRAKE) or drift to a stop (FLOAT) when told to stop.
     *
     * A few things to note:
     *  - The name above in quotes in the hardware map line MUST match the name of one of
     *    the ports in the robot's hardware map. (We will learn what the hardware map is
     *    later on.)
     *  - The `setDirection()` line has two options: `FORWARD` and `REVERSE`. `REVERSE`
     *    reverses the direction of the motor, turning counterclockwise with a positive
     *    power and clockwise with a negative power. This is important when you have motors
     *    pointing in opposite directions and you don't want to hassle with making the power
     *    value for one negative.
     *
     * Copy and paste the above code underneath the "Initialization" comment and move on to
     * the next step.
     *
     */
    /////////////////////////////////////////////////////////////////////////////////////////
    /**
     * ) Step 3: Setting the Power
     * At this point, you have declared your motor and initialized it, getting it ready to
     * be moved. Now you have to actually move it! Luckily, doing so is fairly simple. All
     * you have to do is use `setPower()`:
         motor.setPower(power);
     *
     * The power of a motor is a number that ranges anywhere from 1 to -1, functioning as a
     * percentage of 100%. For example, 1 would set the power to 100%, while 0.5 would set
     * it to 50%.
     *
     * Right underneath the "Do Stuff Here!" comment, I want you to set the power of your
     * motor to 100%, and see what happens. Then I want you to experiment with different
     * power values. Try getting it to stop, or move backwards. If you want to call
     * multiple `setPower()` commands one after another, you can type `sleep()` between each
     * `setPower()`:
         sleep(milliseconds);
     *
     * This will make the program wait that many milliseconds before moving on to the next
     * line of code. Remember that there are 1000 milliseconds in a second; if you don't,
     * you might make the program wait 5 thousandths of a second instead of 5 seconds!
     *
     * You may also try changing `FORWARD` in initialization to `REVERSE`, and also `BRAKE`
     * to `FLOAT`. Play with the code to see what you can do!
     *
     */
    /////////////////////////////////////////////////////////////////////////////////////////
    /**
     * ) Coding Challenge: Coding a Chassis
     * Now that you have learned how to code individual motors, it is time to learn how to
     * code a chassis. For those who don't know, a chassis is the base of a robot with wheels
     * on it, allowing it to drive. Chassis come in many different shapes and sizes, but for
     * this program we will be using a square one with 4 motors and 4 wheels, two on each
     * side.
     *
     * For this challenge, there are a few basic rules you will need to follow:
     *  - There must be four motors.
     *  - Each motor in their hardware map line must be named one of the following:
     *    ~ WheelFL
     *    ~ WheelFR
     *    ~ WheelBL
     *    ~ WheelBR
     *  - You must make the robot move forward, stop, go backward, and stop, each for 3 seconds.
     *
     * ) OPTIONAL CHALLENGE: Find a way to make the robot turn, and have it drive in a square.
     *
     * Once you are finished, check the TestingMotorsCompleted.java file and compare your work!
     *
     */
    /////////////////////////////////////////////////////////////////////////////////////////


    /*
     * Declare Hardware
     */



    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialization
         */


        telemetry.addData("=^D", "Ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {      // If we've stopped the robot, stop the program
            if (!opModeIsActive()) {
                break;
            }

            /*
             * Do stuff here!
             */



            break; // End the program once it has finished
        }
    }


}
