package org.firstinspires.ftc.blackswan;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleopRed")

public class TeleopRed extends LinearOpMode {

    double MAX_SPEED = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft, backLeft, frontRight, backRight, arm, carousel, intake;

        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        carousel = hardwareMap.get(DcMotor.class, "carousel");

        arm = hardwareMap.get(DcMotor.class, "arm");

        intake = hardwareMap.get(DcMotor.class, "intake");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        //telemetry testing delete later!!!
        String detection = "none";
        while(opModeIsActive()) {
            //turn with right stick
//            telemetry.addData("left stick value x", gamepad1.left_stick_x);
//            telemetry.addData("left stick value y", gamepad1.left_stick_y);
            telemetry.addData("left stick value x", gamepad2.left_stick_x);
            telemetry.addData("left stick value y", gamepad2.left_stick_y);
            telemetry.addData("detection", detection);
            telemetry.update();
            if (gamepad1.right_stick_x > 0.1) {
                telemetry.addData("positive", gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.right_stick_x < -0.1) {
                telemetry.addData("negative", gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.left_stick_x < -0.25 && gamepad1.left_stick_y < -0.25){
                //move UpLeft
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(0);
                frontLeft.setPower(0);
                detection = "UpLeft";
            } else if (gamepad1.left_stick_x > 0.25 && gamepad1.left_stick_y < -0.25){
                //move UpRight
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(0);
                frontRight.setPower(0);
                detection = "UpRight";
            } else if (gamepad1.left_stick_x > 0.25 && gamepad1.left_stick_y > 0.25){
                //move DownRight
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(0);
                frontLeft.setPower(0);
                detection = "DownRight";
            } else if (gamepad1.left_stick_x < -0.25 && gamepad1.left_stick_y > 0.25){
                //move DownLeft
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(0);
                frontRight.setPower(0);
                detection = "DownLeft";
            } else if (gamepad1.left_stick_y > 0.1){
                //move Down
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                detection = "Down";
            } else if (gamepad1.left_stick_y < -0.1){
                //move Up
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                detection = "Up";
            } else if (gamepad1.left_stick_x > 0.1){
                //move Right
                frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_x * MAX_SPEED* -1);
                frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
                detection = "Right";
            } else if (gamepad1.left_stick_x < -0.1){
                //move Left
                frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
                detection = "Left";
            }else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                detection = "None";
            }
            if (gamepad2.dpad_up) { //up
                arm.setTargetPosition(1300);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);
                while (arm.isBusy() && opModeIsActive()) {
                }
            }
            if (gamepad2.dpad_left) { //middle
                arm.setTargetPosition(700);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);
                while (arm.isBusy() && opModeIsActive()) {
                }
            }
            if (gamepad2.dpad_right) { //low
                arm.setTargetPosition(400);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);
                while (arm.isBusy() && opModeIsActive()) {
                }
            }

            if (gamepad2.dpad_down ) { //floor
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.3);
                while (arm.isBusy() && opModeIsActive()) {

                }
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            if(gamepad2.right_trigger > 0.1){
                intake.setPower(-1);
            } else if(gamepad2.left_trigger > 0.1){
                intake.setPower(1);
            } else{
                intake.setPower(0);
            }


            telemetry.update();

            turnDuck(carousel);
        }

    }

    protected void turnDuck(DcMotor carousel){
        if(gamepad2.right_bumper){
            carousel.setPower(-0.5 );
        } else {
            carousel.setPower(0);
        }
    }
}

