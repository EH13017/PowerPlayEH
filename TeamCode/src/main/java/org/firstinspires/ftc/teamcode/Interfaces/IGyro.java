package org.firstinspires.ftc.teamcode.Interfaces;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public interface IGyro {
    public double GetAngle();
    public void ResetAngle();
    public int GetHeadingEH();
    public void ResetHeadingEH();
}
