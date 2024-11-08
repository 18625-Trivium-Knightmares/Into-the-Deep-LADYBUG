package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp (name = "TeleOp", group = "TELEOP")
public class TestingSlides extends LinearOpMode {

    DcMotor slide;
    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotor.class, "slide");
        waitForStart();
        while (opModeIsActive()) {
            slide.setPower(gamepad1.left_stick_y);
        }

    }
}