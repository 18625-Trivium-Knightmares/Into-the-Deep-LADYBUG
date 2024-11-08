package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "ServoTest", group = "TELEOP")
public class ServoTest extends LinearOpMode {
    Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "gripper");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                servo.setPosition(0.05);
            } else if (gamepad2.right_bumper) {
                servo.setPosition(0.45);
            }
        }
    }
}
