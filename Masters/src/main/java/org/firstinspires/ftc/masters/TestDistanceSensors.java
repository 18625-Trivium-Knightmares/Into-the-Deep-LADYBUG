package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
//@Autonomous(name = "Distance sensors test thing", group="test")
public class TestDistanceSensors extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode () {
        robot = new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        robot.distanceSensorStuff(.2);
        while (true) {
            telemetry.addData("Left sensor distance = ", robot.distanceSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Right sensor distance = ", robot.distanceSensorRight.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }
}
