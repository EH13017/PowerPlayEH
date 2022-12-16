package org.firstinspires.ftc.teamcode.Lessons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Autonomous(name = "Testing Motors - Completed", group = "Lessons")
public class Lesson01_TestingMotorsCompleted extends LinearOpMode {

    /*
     * Declare Hardware
     */
    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialization
         */

        motor = hardwareMap.dcMotor.get("WheelFR");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

            motor.setPower(1.0);
            sleep(3000);
            motor.setPower(0);
            sleep(3000);
            motor.setPower(-1.0);
            sleep(3000);
            motor.setPower(0);



            break; // End the program once it has finished
        }
    }


}
