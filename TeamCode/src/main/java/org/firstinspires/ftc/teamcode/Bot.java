package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Bot {

    //OpMode Declaration
    private LinearOpMode opMode;

    //Motor Declaration
    private DcMotor leftMotorFront;
    private DcMotor rightMotorFront;
    private DcMotor leftMotorBack;
    private DcMotor rightMotorBack;

    private DcMotor rightLift;
    private DcMotor leftLift;

    private CRServo topIntake;
    private CRServo bottomIntake;


    //Statistics for measurements



    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Constructor for Bot object
     * @param opMode
     */
    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode.hardwareMap);
    }

    /**
     * Initialize hardware mapping for robot
     * @param map
     */
    public void init(HardwareMap map){

        //Connecting declared motors to classes of DcMotors and respected names
        leftMotorFront = map.get(DcMotor.class, "left_front");
        leftMotorBack = map.get(DcMotor.class, "left_back");
        rightMotorFront = map.get(DcMotor.class, "right_front");
        rightMotorBack = map.get(DcMotor.class, "right_back");
        leftLift = map.get(DcMotor.class, "left_lift");//giveing the motors a name for codeing
        rightLift = map.get(DcMotor.class, "right_lift");

        //Set RunModes for Encoder Usage
        /*
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */

        //Set Direction of each Motors
        // switch REVERSE and FORWARD if controls are opposite
        // This is set for Mechanum drive
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);//this is because the motors are probalby faceing each other

        //Servos for intake on the map
        topIntake = map.get(CRServo.class, "top_intake");
        bottomIntake = map.get(CRServo.class, "bottom_intake");
    }

    /**
     * Set drive train power for mechanum drive
     * @param frontLeftPower
     * @param backLeftPower
     * @param frontRightPower
     * @param backRightPower
     */
    public void setDriveTrain(
           double frontLeftPower, double backLeftPower,
           double frontRightPower, double backRightPower
    ) {
       leftMotorFront.setPower(frontLeftPower);
       leftMotorBack.setPower(backLeftPower);
       rightMotorFront.setPower(frontRightPower);
       rightMotorBack.setPower(backRightPower);
    }

    public void setLift(
            double liftPower
    ){
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    public void setIntakePosition(
            double intakePower
    ) {
        topIntake.setPower(intakePower);
        bottomIntake.setPower(-intakePower);
    }


    //Drive Encoder
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.09449;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double INCHES_PER_DEGREE = 0.1744;
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        newfrontLeftTarget = leftMotorFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newfrontRightTarget = rightMotorFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        newbackLeftTarget = leftMotorBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newbackRightTarget = rightMotorBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        leftMotorFront.setTargetPosition(newfrontLeftTarget);
        rightMotorFront.setTargetPosition(newfrontRightTarget);
        leftMotorBack.setTargetPosition(newbackLeftTarget);
        rightMotorBack.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftMotorFront.setPower(Math.abs(speed));
        rightMotorFront.setPower(Math.abs(speed));
        leftMotorBack.setPower(Math.abs(speed));
        rightMotorBack.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftMotorFront.isBusy() || rightMotorFront.isBusy() || leftMotorBack.isBusy() || rightMotorBack.isBusy())) {
            opMode.telemetry.addData("frontLeftEncoder", leftMotorFront.getCurrentPosition());
            opMode.telemetry.addData("frontRightEncoder", rightMotorFront.getCurrentPosition());
            opMode.telemetry.addData("backLeftEncoder", leftMotorBack.getCurrentPosition());
            opMode.telemetry.addData("backRightEncoder", rightMotorBack.getCurrentPosition());
            opMode.telemetry.update();
        }


        // Stop all motion;
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Turn Encoder
    public void encoderTurn(double speed, double degrees, double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = leftMotorFront.getCurrentPosition() + (int) (-degrees * COUNTS_PER_INCH * INCHES_PER_DEGREE);
            newfrontRightTarget = rightMotorFront.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH * INCHES_PER_DEGREE);
            newbackLeftTarget = leftMotorBack.getCurrentPosition() + (int) (-degrees * COUNTS_PER_INCH * INCHES_PER_DEGREE);
            newbackRightTarget = rightMotorBack.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH * INCHES_PER_DEGREE);

            leftMotorFront.setTargetPosition(newfrontLeftTarget);
            rightMotorFront.setTargetPosition(newfrontRightTarget);
            leftMotorBack.setTargetPosition(newbackLeftTarget);
            rightMotorBack.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotorFront.setPower(Math.abs(speed));
            rightMotorFront.setPower(Math.abs(speed));
            leftMotorBack.setPower(Math.abs(speed));
            rightMotorBack.setPower(Math.abs(speed));

            // Stop all motion;
            leftMotorFront.setPower(0);
            rightMotorFront.setPower(0);
            leftMotorBack.setPower(0);
            rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderStrafe(double speed, double distance, double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = leftMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newfrontRightTarget = rightMotorFront.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newbackLeftTarget = leftMotorBack.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newbackRightTarget = rightMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            leftMotorFront.setTargetPosition(newfrontLeftTarget);
            rightMotorFront.setTargetPosition(newfrontRightTarget);
            leftMotorBack.setTargetPosition(newbackLeftTarget);
            rightMotorBack.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotorFront.setPower(Math.abs(speed));
            rightMotorFront.setPower(Math.abs(speed));
            leftMotorBack.setPower(Math.abs(speed));
            rightMotorBack.setPower(Math.abs(speed));

            // Stop all motion;
            leftMotorFront.setPower(0);
            rightMotorFront.setPower(0);
            leftMotorBack.setPower(0);
            rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
