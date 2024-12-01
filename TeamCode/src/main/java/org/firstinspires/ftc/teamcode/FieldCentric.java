package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Field centric", group = "TELEOP")
public class FieldCentric extends LinearOpMode {

    DcMotor FR, FL, BR, BL, arm, slide;
    Servo claw;

    IMU imu;
    IMU.Parameters myIMUparameters;

    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        BR = hardwareMap.get(DcMotor.class, "rightBack");
        BL = hardwareMap.get(DcMotor.class, "leftBack");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slide = hardwareMap.get(DcMotor.class, "arm");
        arm = hardwareMap.get(DcMotor.class, "slide");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.get(Servo.class, "claw");

        // IMU
        imu = hardwareMap.get(IMU.class, "imu"); // Initializing IMU in Drivers Hub
        // Reconfiguring IMU orientation
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        waitForStart();

        int setArm = 10;
        int setSlide = -1;
        arm.setTargetPosition(setArm);
        while (opModeIsActive()) {

            /**
             * Game pad 1
             */
            fieldCentric();


            /**
             * Game pad 2
             */

            if (gamepad2.left_bumper) {
                claw.setPosition(0.77);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(1.0);
            }

            if (gamepad2.dpad_up/* && arm.getCurrentPosition() < 1000*/) {
                setArm += 15;
                sleep(5);
            } else if (gamepad2.dpad_down && arm.getCurrentPosition() > 10) {
                setArm -= 15;
                sleep(5);
            }

            /*if (arm.getCurrentPosition() > 1000) {
                setArm = 1000;
            } else */if (arm.getCurrentPosition() < 10) {
                setArm = 10;
            }

            if (slide.getCurrentPosition() > -1) {
                setSlide = -1;
            } else if (slide.getCurrentPosition() < -4000) {
                setSlide = -4000;
            }

            if (gamepad2.right_trigger > 0 && slide.getCurrentPosition() < -1) {
                setSlide += 50;
                sleep(5);
            } else if (gamepad2.left_trigger > 0 && slide.getCurrentPosition() > -4000) {
                setSlide -= 50;
                sleep(5);
            }

            // presets
            if (gamepad2.a) {
                setArm = 10;
            } else if (gamepad2.b) { // low chamber
                setArm = 240;
            } else if (gamepad2.x) { // low basket
                setArm = 300;
            } else if (gamepad2.y) { // high chamber
                setArm = 310;
            }




            telemetry.addLine("arm:" + arm.getCurrentPosition());
            telemetry.addLine("slide:" + slide.getCurrentPosition());
            telemetry.update();

            arm.setTargetPosition(setArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.8);

            slide.setTargetPosition(setSlide);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.8);

        }

    }
    /** field centric ahh stuff
     *
     */
    public void fieldCentric() {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double vertical = -gamepad1.left_stick_y * 1;
        double horizontal = gamepad1.left_stick_x * 1;
        double pivot = gamepad1.right_stick_x * 1;
        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(pivot), 1);

        if (gamepad1.right_trigger > 0) {
            vertical = -gamepad1.left_stick_y * 0.5;
            horizontal = gamepad1.left_stick_x * 0.5;
            pivot = gamepad1.right_stick_x * 0.6;
        }

        // Kinematics (Counter-acting angle of robot's heading)
        double newVertical = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);
        double newHorizontal = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);

        // Setting Field Centric Drive
        FL.setPower((newVertical + newHorizontal + pivot)/denominator);
        FR.setPower((newVertical - newHorizontal - pivot)/denominator);
        BL.setPower((newVertical - newHorizontal + pivot)/denominator);
        BR.setPower((newVertical + newHorizontal - pivot)/denominator);
    }
}