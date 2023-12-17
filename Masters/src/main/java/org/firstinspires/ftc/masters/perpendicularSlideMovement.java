package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "perpendicularSlideMovement")
public class perpendicularSlideMovement extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        
        DcMotor leftHorizontal = hardwareMap.dcMotor.get("gpSlideLeft");
        DcMotor rightHorizontal = hardwareMap.dcMotor.get("gpSlideRight");
        
//        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.dpad_up){
                leftHorizontal.setPower(.2);
                rightHorizontal.setPower(.2);
            } else if (gamepad1.dpad_down) {
                leftHorizontal.setPower(-.2);
                rightHorizontal.setPower(-.2);
            } else {
                leftHorizontal.setPower(0);
                rightHorizontal.setPower(0);
            }


        }
    }
}