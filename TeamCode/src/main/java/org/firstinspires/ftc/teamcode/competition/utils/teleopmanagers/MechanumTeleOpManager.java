package org.firstinspires.ftc.teamcode.competition.utils.teleopmanagers;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.Mechanum;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;

public class MechanumTeleOpManager extends TeleOpManager {

    private final Mechanum MECHANUM;
    private final Motor SPINNER, LIFT;

    public MechanumTeleOpManager(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, GamepadFunctions function1, GamepadFunctions function2) {
        super(gamepad1, function1, gamepad2, function2);
        Motor rt = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        Motor rb = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_2), DcMotorSimple.Direction.FORWARD);
        Motor lt = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        Motor lb = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_2), DcMotorSimple.Direction.FORWARD);
        MECHANUM = new Mechanum(telemetry, rt, rb, lt, lb);
        SPINNER = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER), DcMotorSimple.Direction.FORWARD);
        LIFT = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT), DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void main() {
        if(getGamepad1Functions().hasF1()) {
            // do the math
            double y = -getGamepad1().left_stick_y;
            double x = getGamepad1().left_stick_x * 1.1;
            double rx = getGamepad1().right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double rightTopPower = (y - x - rx) / denominator;
            double rightBottomPower = (y + x - rx) / denominator;
            double leftTopPower = (y + x + rx) / denominator;
            double leftBottomPower = (y - x + rx) / denominator;
            // normal drive code
            MECHANUM.driveWithEncoder((int) Range.clip(rightTopPower * 100, -100, 100), (int) Range.clip(rightBottomPower * 100, -100, 100), (int) Range.clip(leftTopPower * 100, -100, 100), (int) Range.clip(leftBottomPower * 100, -100, 100));
        }else if(getGamepad2Functions().hasF1()) {
            // do the math
            double y = -getGamepad2().left_stick_y;
            double x = getGamepad2().left_stick_x * 1.1;
            double rx = getGamepad2().right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double rightTopPower = (y - x - rx) / denominator;
            double rightBottomPower = (y + x - rx) / denominator;
            double leftTopPower = (y + x + rx) / denominator;
            double leftBottomPower = (y - x + rx) / denominator;
            // normal drive code
            MECHANUM.driveWithEncoder((int) Range.clip(rightTopPower * 100, -100, 100), (int) Range.clip(rightBottomPower * 100, -100, 100), (int) Range.clip(leftTopPower * 100, -100, 100), (int) Range.clip(leftBottomPower * 100, -100, 100));
        }
        if(getGamepad1Functions().hasF2()) {
            SPINNER.driveWithEncoder((int) Range.clip((getGamepad1().left_trigger - getGamepad1().right_trigger) * 100, -100, 100));
            int liftSpeed1 = getGamepad1().left_bumper ? 50 : 0;
            int liftSpeed2 = getGamepad1().right_bumper ? -50 : 0;
            int liftSpeed = liftSpeed1 - liftSpeed2;
            LIFT.driveWithEncoder(liftSpeed);
        }else if(getGamepad2Functions().hasF2()) {
            int liftSpeed1 = getGamepad2().left_bumper ? 50 : 0;
            int liftSpeed2 = getGamepad2().right_bumper ? -50 : 0;
            int liftSpeed = liftSpeed1 - liftSpeed2;
            LIFT.driveWithEncoder(liftSpeed);
            SPINNER.driveWithEncoder((int) Range.clip((getGamepad2().left_trigger - getGamepad2().right_trigger) * 100, -100, 100));
        }
    }

    @Override
    public void stop() {
        MECHANUM.stop();
    }

}
