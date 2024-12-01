package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "righto", group = "auto")
public class Righto extends LinearOpMode {
    DcMotor FR, FL, BR, BL;
    Servo claw;

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

        claw = hardwareMap.get(Servo.class, "claw");

        claw.setPosition(0.63);

        waitForStart();

        FR.setPower(-0.5);
        FL.setPower(0.5);
        BR.setPower(0.5);
        BL.setPower(-0.5);

        sleep(2500);

        claw.setPosition(0.4);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);

    }
}
