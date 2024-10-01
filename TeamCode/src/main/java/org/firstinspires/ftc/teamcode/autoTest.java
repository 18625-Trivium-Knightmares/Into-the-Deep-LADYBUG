package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp (name = "Adam :)")
public class autoTest extends OpMode {

    private DcMotor backLeft; //making back left motor
    private DcMotor frontLeft; //making front left motor
    private DcMotor backRight; //making back right motor
    private DcMotor frontRight; //making front right motor
    private static double powerInput = 1; //making power variable
    private DcMotor intake; //making intake motor

    @Override
    public void loop() {

    } //idk

    @Override
    public void init() { //initialize function
        backLeft = hardwareMap.get(DcMotor.class, "motorLeft"); //mapping motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "motorRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); //reversing left side cuz mecanum wheels
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        powerInput = gamepad1.right_stick_y; //mapping power variable to right stick up and down
    }
    public void goForward() { //go forward
        backLeft.setPower(powerInput);
        frontLeft.setPower(powerInput);
        backRight.setPower(powerInput);
        frontRight.setPower(powerInput);
    }
    public  void goBackward() { //go backward
        backLeft.setPower(-1 * powerInput);
        frontLeft.setPower(-1 * powerInput);
        backRight.setPower(-1 * powerInput);
        frontRight.setPower(-1 * powerInput);
    }
    public void goRight() { //go right
        frontRight.setPower(-1 * powerInput);
        frontLeft.setPower((powerInput));
        backRight.setPower(powerInput);
        backLeft.setPower(-1 * powerInput);
    }
    public void goLeft() { //go left
        frontLeft.setPower(-1 * powerInput);
        frontRight.setPower(powerInput);
        backLeft.setPower(powerInput);
        backRight.setPower(-1 * powerInput);
    }
    public void goForwardRight() { // go foward and right
        frontLeft.setPower(powerInput);
        backRight.setPower(powerInput);
    }
    public void goFowardLeft() { //go forward and left
        frontRight.setPower(powerInput);
        backLeft.setPower(powerInput);
    }
    public void goBackwardRight() { //go backward and right
        frontLeft.setPower(-1 * powerInput);
        backRight.setPower(-1 * powerInput);
    }
    public void goBackwardLeft() { //go backward and left
        frontRight.setPower(-1 * powerInput);
        backLeft.setPower(-1 * powerInput);
    }
    public void intakeSpinIn() { //spin intake wheel to put brick in
        intake.setPower(powerInput);
    }
    public void intakeSpinOut() { //spin intake wheel to spit brick out
        intake.setPower(-1 * powerInput);
    }
}
