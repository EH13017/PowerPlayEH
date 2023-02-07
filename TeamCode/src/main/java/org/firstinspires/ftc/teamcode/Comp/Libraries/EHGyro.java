package org.firstinspires.ftc.teamcode.Comp.Libraries;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Interfaces.IGyro;

public class EHGyro implements IGyro {
    public ModernRoboticsI2cGyro Gyro;
    private Orientation _lastAngles = new Orientation();
    private double _globalAngle;

    public EHGyro(ModernRoboticsI2cGyro gyro){
        Gyro = gyro;
    }


    /**
     * Get current cumulative angle rotation from last reset.
     * https://stemrobotics.cs.pdx.edu/node/7268
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    @Override
    public double GetAngle() {
        /* We experimentally determined the Z axis is the axis we want to use for heading angle.
         * We have to process the angle because the imu works in euler angles so the Z axis is
         * returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
         * 180 degrees. We detect this transition and track the total cumulative angle of rotation. */


        /* Read dimensionalized data from the gyro. This gyro can report angular velocities
         * about all three axes. Additionally, it internally integrates the Z axis to
         * be able to report an absolute angular Z orientation. */
        Orientation angles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - _lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        _globalAngle += deltaAngle;

        _lastAngles = angles;

        return _globalAngle;
    }

    @Override
    public void ResetAngle() {
        _lastAngles = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        _globalAngle = 0;
    }

    /**
     * Get current cumulative angle heading.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    @Override
    public int GetHeadingEH(){
        return Gyro.getIntegratedZValue();
    }

    @Override
    public void ResetHeadingEH() {
        Gyro.resetZAxisIntegrator();
    }
}
