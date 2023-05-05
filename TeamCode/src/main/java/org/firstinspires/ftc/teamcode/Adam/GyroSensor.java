package org.firstinspires.ftc.teamcode.Adam;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Comp.Libraries.AprilTag;
import org.firstinspires.ftc.teamcode.Comp.Libraries.DriveWithoutEncoders;
import org.firstinspires.ftc.teamcode.Comp.Libraries.EHGyro;
import org.firstinspires.ftc.teamcode.Comp.Libraries.PIDController;
import org.firstinspires.ftc.teamcode.Comp.Libraries.Rotate;
import org.firstinspires.ftc.teamcode.Interfaces.IDrive;
import org.firstinspires.ftc.teamcode.Interfaces.IGyro;
import org.firstinspires.ftc.teamcode.Interfaces.IRotate;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name = "Gyro Sensor", group = "Testing")
public class GyroSensor extends LinearOpMode{
    // Define a variable for our gyro sensor
    ModernRoboticsI2cGyro Gyro;

    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;

    @Override
    public void runOpMode() {
        telemetry.addData("Gyro",Gyro.rawY());
    }
}
