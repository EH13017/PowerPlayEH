package org.firstinspires.ftc.teamcode.Interfaces;

public interface IDrive {
    public enum Direction {
        FORWARD,
        BACKWARD
    }
    public void Forward(double distanceInch, double power);
    public void Backward(double distanceInch, double power);
    public void Straight(Direction direction, double distanceInch, double power);
    public void ShowTelemetry();
}
