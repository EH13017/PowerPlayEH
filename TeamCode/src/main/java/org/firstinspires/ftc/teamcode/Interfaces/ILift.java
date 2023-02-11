package org.firstinspires.ftc.teamcode.Interfaces;

//import org.firstinspires.ftc.teamcode.freightfrenzycomp.Lift;

public interface ILift {
//    public void SetTargetHubLevel(Lift.HubLevel hubLevel);
//    public Lift.HubLevel DetermineHubLevel(float positionCoordinate);
//    public Lift.HubLevel ReadBarCode();
    public void LiftToLevel();
    public void BucketIn();
    public void BucketOut();
    public void GetTelemetry();
}
