package org.firstinspires.ftc.teamcode.drive.auto.deprecated;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmodes.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(group="Blue")
@Disabled
public class TestAuto extends LinearOpMode
{
    //INTRODUCE VARIABLES HERE

    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive drive = null;
    Trajectory traj1ParkRight, traj2ParkRight, traj1ParkMiddle, traj2ParkMiddle, traj1ParkLeft, traj2ParkLeft;

    private Servo claw, LeftServo, RightServo = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("left_front");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("left_back");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("right_front");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("right_back");

        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");

        RightServo.setDirection(Servo.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");

        drive = new SampleMecanumDrive(hardwareMap);

        motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);

        Pose2d startPose = new Pose2d(35, 62.5, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);

        traj1ParkRight = drive.trajectoryBuilder(startPose, false)
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);

                })

                .lineToLinearHeading(new Pose2d(13.58, 60, Math.toRadians(90)))

                .addDisplacementMarker(() -> {
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);
                })

                .build();

        traj2ParkRight = drive.trajectoryBuilder(traj1ParkRight.end(), false)
                .lineToLinearHeading(new Pose2d(13.58, 24, Math.toRadians(90)))

                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(0.6);
                    RightServo.setPosition(0.6);

                })
                .build();

        traj1ParkMiddle = drive.trajectoryBuilder(startPose, false)
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);

                })

                .lineToLinearHeading(new Pose2d(40, 55, Math.toRadians(90)))

                .addDisplacementMarker(() -> {
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);

                })


                .build();

        traj2ParkMiddle = drive.trajectoryBuilder(traj1ParkMiddle.end(), false)
                .lineToLinearHeading(new Pose2d(40, 24, Math.toRadians(90)))

                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(0.6);
                    RightServo.setPosition(0.6);

                })


                .build();

        traj1ParkLeft = drive.trajectoryBuilder(startPose, false)
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);
                })

                .lineToLinearHeading(new Pose2d(66, 62, Math.toRadians(90)))
                .build();

        traj2ParkLeft = drive.trajectoryBuilder(traj1ParkLeft.end(), false)
                .lineToLinearHeading(new Pose2d(66, 24, Math.toRadians(90)))

                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(0.6);
                    RightServo.setPosition(0.6);
                })
                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }





        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
            if(tagOfInterest.id == RIGHT) {
                drive.followTrajectory(traj1ParkRight);
                drive.followTrajectory(traj2ParkRight);


            }else if(tagOfInterest.id == LEFT) {
                drive.followTrajectory(traj1ParkLeft);
                drive.followTrajectory(traj2ParkLeft);


            } else {
                drive.followTrajectory(traj1ParkMiddle);
                drive.followTrajectory(traj2ParkMiddle);


            }
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);




    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
