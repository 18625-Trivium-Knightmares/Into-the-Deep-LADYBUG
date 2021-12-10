package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="freightFrenzy")
public class InitialMecanumTeleOp extends LinearOpMode {


    RobotClass robot;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    DcMotor intakeMotor = null;
    DcMotor linearSlideMotor = null;

    Servo dumpServo = null;

    DcMotor carouselMotor = null;
    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;
    final int TOP_ENCODER_VALUE = 1700;
    final int MIDDLE_ENCODER_VALUE = 1300;
    final int BOTTOM_ENCODE_VALUE = 800;
    // operational constants
    double joyScale = 1;
    double motorMax = 0.99; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    int linearSlideTolerance = 5;



    private enum linearSlideTargets {
        TOP,
        MIDDLE,
        BOTTOM,
        BASE
    }

    public enum linearSlidePositions {
        TOP,
        MIDDLE,
        BOTTOM,
        BASE
    }

    linearSlideTargets linearSlideTarget = linearSlideTargets.BASE;
    linearSlidePositions linearSlidePos = linearSlidePositions.BASE;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new RobotClass(hardwareMap, telemetry, this);

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");
        carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlide");

        dumpServo = hardwareMap.servo.get("dump");

        // Set the drive motor direction:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // These polarities are for the Neverest 20 motors
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)
//        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean carouselOn = false; //Outside of loop()
        boolean intakeOn = false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            intakeMotor.setPower(0);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double leftFrontPower = y + x - rx;
            double leftRearPower = y - x - rx;
            double rightFrontPower = y - x + rx;
            double rightRearPower = y + x + rx;

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                double max;
                max = Math.max(leftFrontPower, leftRearPower);
                max = Math.max(max, rightFrontPower);
                max = Math.max(max, rightRearPower);

                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            leftFrontMotor.setPower(leftFrontPower);
            leftRearMotor.setPower(leftRearPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightRearMotor.setPower(rightRearPower);


            if(gamepad2.y && !carouselOn) {
                if(carouselMotor.getPower() != 0) carouselMotor.setPower(0);
                else carouselMotor.setPower(.6);
                carouselOn = true;
            } else if(!gamepad2.y) carouselOn = false;

            if(gamepad2.a) {
                if(intakeMotor.getPower() != 0) intakeMotor.setPower(0);
                else
                    intakeMotor.setPower(-.8);
                intakeOn = true;
            } else if (gamepad2.b) {
                intakeMotor.setPower(.8);
                intakeOn = false;
            } else if (gamepad2.x){
                intakeOn = false;
                intakeMotor.setPower(0);
            }


            if (gamepad2.dpad_up) {
//                Top scoring
                linearSlideTarget = linearSlideTargets.TOP;
                intakeMotor.setPower(0);
                intakeOn = false;
                linearSlideMotor.setTargetPosition(TOP_ENCODER_VALUE);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.8);//.4
            }

            if (gamepad2.dpad_left) {
//                Middle scoring
                linearSlideTarget = linearSlideTargets.MIDDLE;
                intakeMotor.setPower(0);
                intakeOn = false;
                linearSlideMotor.setTargetPosition(MIDDLE_ENCODER_VALUE);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.8);//.4
            }

            if (gamepad2.dpad_down) {
//                Low scoring
                linearSlideTarget = linearSlideTargets.BOTTOM;
                intakeMotor.setPower(0);
                intakeOn = false;
                linearSlideMotor.setTargetPosition(BOTTOM_ENCODE_VALUE);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlideMotor.setPower(.8);//.4
            }

            if (gamepad2.left_trigger >= .35) {
                if (linearSlideMotor.getCurrentPosition() >= 500) {
//                    dump
                    dumpServo.setPosition(.8);//1.5
                    sleep(1800);

                    dumpServo.setPosition(.41);//0.75
                    linearSlideTarget = linearSlideTargets.BASE;
                    linearSlideMotor.setTargetPosition(0);
                    linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlideMotor.setPower(-.8);//-.4
                }
            }

            linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (linearSlideTarget == linearSlideTargets.BASE) {
                if (linearSlideMotor.getCurrentPosition() <= 10) {
                    linearSlideMotor.setPower(0);
                    linearSlidePos = linearSlidePositions.BASE;
                    intakeOn = true;
                   // intakeMotor.setPower(.8);
                }
            }

        }
    }
}