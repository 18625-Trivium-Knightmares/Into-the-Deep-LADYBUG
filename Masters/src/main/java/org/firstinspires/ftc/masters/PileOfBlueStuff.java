package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name = "Pile of blue stuff")
public class PileOfBlueStuff extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() {
        robot = new RobotClass(hardwareMap,telemetry,this);
        robot.openCVInnitShenanigans();
        EasyOpenCVIdentifyShippingElement.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;

        waitForStart();
        // Read the bar code with open CV

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = robot.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }

        // Read the bar code with open CV


        /*
        robot.strafeRight(0.5,2);
        robot.forward(0.3,0.6);
        //deposit shipping element.

        robot.strafeLeft(0.3,2);
        robot.turnToHeading(0.3,-90,3);
        robot.forward(0.3,4);
        robot.turnToHeading(0.3,0,3);
        robot.backwards(0.3,1);
        robot.forward(0.3,1);
        robot.turnToHeading(0.3,90,3);
        robot.forward(0.5,4.6);
        robot.turnToHeading(0.3,0,3);
        // deposit shipping element

        robot.turnToHeading(0.3,-90,3);
        robot.forward(0.3,4);
        robot.turnToHeading(0.3,0,3);
        robot.backwards(0.3,1);
        robot.forward(0.3,1);
        robot.turnToHeading(0.3,90,3);
        robot.forward(0.5,4.6);
        robot.turnToHeading(0.3,0,3);
*/
    }
}