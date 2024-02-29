package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive2024;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;
import org.firstinspires.ftc.teamcode.fileUtils;

@Autonomous(name = "BlueRight", group = "")
public class BlueRight extends LinearOpMode {
    private static final int NUMLOOPS = 3 ;
    //test1

    private MecanumDrive2024 drive;
    private actuatorUtils utils;
    private moveUtils move;

    private Servo dump = null; //Located on Expansion Hub- Servo port 0
    private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    private Servo elbow = null; //Located on Expansion Hub- Servo port 0

    private DcMotor arm = null;
    private DcMotor arm1 = null;


    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public double desiredHeading;

    Orientation angles;
    Acceleration gravity;
    private OpenCvCamera webCam;
    private boolean isCameraStreaming = false;
    Pipeline2023 modifyPipeline = new Pipeline2023(true);

    private int resultROI = 3;

    private boolean done = false;
    private fileUtils fUtils;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive2024(hardwareMap);
        utils = new actuatorUtils();
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        dump = hardwareMap.get(Servo.class, "Dump");
        elbow = hardwareMap.get(Servo.class, "elbow");
        gripper = hardwareMap.get(Servo.class, "gripper");
        Pose2d startPose = new Pose2d(-55, -26,0);
        drive.setPoseEstimate(startPose);
        elbow.setPosition(1);

        //TrajectorySequence aSeq = autoSeq(startPose);


        //Reverse the arm direction so it moves in the proper direction
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm1.setDirection(DcMotor.Direction.REVERSE);
        arm.setPower(0);
        arm1.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fUtils = new fileUtils();
        desiredHeading = getHeading();

        utils.initializeActuator(arm, arm1, gripper, dump, elbow);
        move.initialize(drive, utils);

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;

        initOpenCV();

        utils.dumpClose();

        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        //sleep(5000);
        if (resultROI == 3) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            done = false;
            while (!done && opModeIsActive()) {
                //if (currTime - startTime < 500) {
                //    telemetry.addData("Camera: ", "Waiting to make sure valid data is incoming");
                //} else {
                telemetry.addData("Time Delta: ", (currTime - startTime));
                telemetry.addData("Middle Count", modifyPipeline.getMiddleResult());
                telemetry.addData("Right Count", modifyPipeline.getRightResult());
                resultROI = modifyPipeline.getResultROI();
                if (resultROI == 0) {
                    telemetry.addData("Resulting ROI: ", "Left");
                    done = true;
                } else if (resultROI == 1) {
                    telemetry.addData("Resulting ROI: ", "Middle");
                    done = true;
                } else if (resultROI == 2) {
                    telemetry.addData("Resulting ROI: ", "Right");
                    done = true;
                } else {
                    telemetry.addData("Resulting ROI: ", "Something went wrong.");
                }
                telemetry.addData("Middle Result; ", modifyPipeline.getMiddleResult());
                telemetry.addData("Right Result; ", modifyPipeline.getRightResult());

                //}
                telemetry.update();
                currTime = System.currentTimeMillis();

            }

        }
        telemetry.update();
        done = false;

        //lift arm up
        while (((currTime - startTime) < 30000) && !done && opModeIsActive()) {
            //autoSeq();
            //telemetry.addData("IM at ", getHeading());
            //telemetry.update();
            //if (!isStopRequested())
            //actuatorUtils.armPole(actuatorUtils.ArmLevel.ZERO,false);
            if (resultROI == 0) {
                LeftPath();
            } else if (resultROI == 1) {
                MiddlePath();
            } else {
                RightPath();
            }
            //set arm to lowest position
       /*     while (gripperSensor.getDistance(DistanceUnit.INCH)>2 && !isStopRequested())
            {
                telemetry.addData("ARM Position = ", arm.getCurrentPosition());
                telemetry.update();
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-0.7);
            }
            //disabling and resetting arm
            arm.setPower(0);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
            currTime = System.currentTimeMillis();
            done = true;
        }
        webCam.stopStreaming();
        Pose2d pose = drive.getPoseEstimate();
        fUtils.setPose(pose);
        fUtils.writeConfig(hardwareMap.appContext, this);
        telemetry.addData("Final Heading: ", "Heading: "+ pose.getHeading());
        telemetry.update();
    }

    private void LeftPath()throws InterruptedException {
        move.driveSeq(-40,-30,-180);
        move.driveSeq(-30, -26, -180);
        utils.dumpOpen();
        sleep(500);
        move.driveSeq(-30, -31, -180);
        sleep(500);
        utils.dumpClose();
        sleep(500);
        move.driveSeq(-5, -30, -180);
        move.driveSeq(-5, 45, -90);
        move.driveSeq(-40, 45, 90);
        move.driveToBoard(-40, 56.5, 90);
        move.driveFromBoard(-40, 53.5, 90);
        move.driveSeq(-7, 53.5, 90);
        move.driveSeq(-7, 63, 90);
    }
    private void MiddlePath()throws InterruptedException {
        move.driveSeq(-16.5,-43,-180);
        sleep(500);
        utils.dumpOpen();
        sleep(500);
        move.driveSeq(-16.5, -48, -180);
        sleep(500);
        utils.dumpClose();
        move.driveSeq(-2, -48, -180);
        move.driveSeq(-2, -42, -180);
        move.driveSeq(-2, -38, -90);
        move.driveSeq(-2, 40, -90);
        move.driveSeq(-32, 40, 90);
        move.driveToBoard(-32, 53.25, 90);
        move.driveFromBoard(-32, 51.5, 90);
        move.driveSeq(-6, 52.5, 90);
        move.driveSeq(-6, 69, 90);
    }
    private void RightPath()throws InterruptedException {
        move.driveSeq(-16,-30.5,0);
        sleep(500);
        utils.dumpOpen();
        sleep(500);
        move.driveSeq(-17, -27, 0);
        sleep(500);
        utils.dumpClose();
        sleep(500);
        move.driveSeq(-16, -27, 0);
        move.driveSeq(-5, -27, 0);
        move.driveSeq(-5, -23, -90);
        move.driveSeq(-5, 40, -90);
        move.driveSeq(-21, 40, 90);
        move.driveToBoard(-21, 54, 90);
        move.driveFromBoard(-21, 52,90);
        move.driveSeq(-2, 53, 90);
        move.driveSeq(-2, 63, 90);
    }

    private void initOpenCV() {
       int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        // For a webcam (uncomment below)
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId2);
        // For a phone camera (uncomment below)
        // webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId2);
        webCam.setPipeline(modifyPipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Pipeline: ", "Initialized");
                telemetry.update();
                isCameraStreaming = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Something went wrong :(");
                telemetry.update();
            }
        });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getHeading() {
        double angle = drive.getRawExternalHeading();
        return angle;
    }
    private float convertRad(int input) {
        float x;
        x=input/180f;
        x*=Math.PI;
        x*=(1);
        return x;
    }
}


