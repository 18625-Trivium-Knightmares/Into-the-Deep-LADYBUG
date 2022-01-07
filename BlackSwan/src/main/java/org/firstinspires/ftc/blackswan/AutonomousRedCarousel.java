package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutonomousRedCarousel extends LinearOpMode {
    Robot robot;
    int cubethingwithlevels=0;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap,telemetry,this);
        waitForStart();
        robot.left(.5,.5);
        robot.turnLeft(45,.5);
        robot.left(.5,.5);
        robot.back(.47,.5);
        robot.redCarousel(4000);
        //check the code underneath, wrong places, too jerky
        robot.forward(.47,.5);
        robot.right(.5,.5);
        robot.turnLeft(25,.5);
        robot.pause(10);
        robot.forward(.5,.5);
        robot.pause(10);
        robot.right(.5,.5);
        robot.pause(10);
        robot.forward(.35,.5);
        robot.pause(10);
        robot.turnLeft(.15,.5);
        //robot.pause(100); //replace with color sensor check
        if (robot.colorSensorRight.green() > 100) {
            cubethingwithlevels = 1;

        }
        robot.right(.75,.5);
        if (robot.colorSensorRight.green() > 100) {
            cubethingwithlevels = 2;
        }
        if (cubethingwithlevels == 0 ){
            cubethingwithlevels = 3;
        }
        //robot.pause(100); //replace with color sensor check
        robot.right(2,.5);
        robot.forward(.75,.5);
        robot.armThing(cubethingwithlevels);
        robot.back(.75,.5);
        robot.left(4.25,.75);
        robot.forward(1,.5);




    }
}
