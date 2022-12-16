package org.firstinspires.ftc.teamcode.Interfaces;

public interface IVision {
    public enum SignalZone {
        LEFT,
        CENTER,
        RIGHT,
        NULL
    }

    public void initVuforia();
    public void initTfod();
    public void turnOnCamera();
    public SignalZone locateSignalIcon();
    public SignalZone returnSignalZone(String signalIcon);

}
