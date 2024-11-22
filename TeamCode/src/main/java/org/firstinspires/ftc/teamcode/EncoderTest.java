package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderTest extends LinearOpMode {

    DcMotor arm;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                arm.setPower(1);
            }
        }
    }
}
