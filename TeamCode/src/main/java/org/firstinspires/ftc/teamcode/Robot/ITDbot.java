package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Robot.systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public abstract class ITDbot extends Robot {
    //TODO tune these numbers for ITDbot, these are from Zirka
    public static double derivativeConstantAngleDef = 0.0015;
    public static double proportionalConstantAngleDef = 0.02;


    public void Init(){
        derivativeConstantAngle = derivativeConstantAngleDef;
        proportionalConstantAngle = proportionalConstantAngleDef;
    }
}
