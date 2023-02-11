package org.firstinspires.ftc.teamcode.drive.auto.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmodes.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmodes.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="OnePlusOneLeft", group="Blue")
public class OnePlusOneLeft extends LinearOpMode {
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
    Trajectory traj1, traj2, traj3, traj4, traj5, traj6, traj1Right, traj1Middle, traj2Middle, traj1Left, traj2Left, traj7, traj8, traj9, traj10, traj11, traj12, traj13, traj14, traj15, traj16, traj17, traj18, traj19, traj20, traj21, traj22, traj23, traj24, traj25, trajdropConeLol;

    private Servo claw, LeftServo, RightServo = null;

    private DcMotorEx LeftSlide, RightSlide = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("left_front");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("left_back");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("right_front");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("right_back");

        LeftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RightSlide");

        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");

        RightServo.setDirection(Servo.Direction.REVERSE);
        RightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");

        drive = new SampleMecanumDrive(hardwareMap);

        motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);

        Pose2d startPose = new Pose2d(35, 63.1, Math.toRadians(-90.00));
        drive.setPoseEstimate(startPose);

        traj1 = drive.trajectoryBuilder(startPose, false)
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                })

                .lineToLinearHeading(new Pose2d(16, 57, Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);
                })
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end(), false)
                .lineToLinearHeading(new Pose2d(16, 20, Math.toRadians(-90)), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    extendSlides(1000, 0.85);
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);
                })
                .build();

        traj4 = drive.trajectoryBuilder(traj2.end(), false)
                .lineToLinearHeading(new Pose2d(18, 8, Math.toRadians(-50)))
                .build();

        traj5 = drive.trajectoryBuilder(traj4.end(), false)
                .back(5)
                .addDisplacementMarker(() -> {
                    claw.setPosition(0);
                })
                .build();

        traj6 = drive.trajectoryBuilder(traj5.end(), false)
                .lineToLinearHeading(new Pose2d(13, 10.5, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
                    retractSlides(0.3); // go down faster or it will time out
                })
                .build();

        traj7 = drive.trajectoryBuilder(traj6.end(), false)
                .lineToLinearHeading(new Pose2d(13.1, 9.5, Math.toRadians(0)))
                .build();

        traj8 = drive.trajectoryBuilder(traj7.end(), false)
                .lineToLinearHeading(new Pose2d(50, 9.5, Math.toRadians(0)))
                .build();

        traj9 = drive.trajectoryBuilder(traj8.end(), false)
                .lineToLinearHeading(new Pose2d(53, 9.5, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    claw.setPosition(0);

                })
                .build();

        traj10 = drive.trajectoryBuilder(traj9.end(), false)
                .addDisplacementMarker(() -> {
                    LeftServo.setPosition(0.9);
                    RightServo.setPosition(0.9);
                    extendSlides(90, 0.9);
                })
                .forward(6.5, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        traj11 = drive.trajectoryBuilder(traj10.end(), false)
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                })
                .back(0.1, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        traj12 = drive.trajectoryBuilder(traj11.end(), false)
                .addDisplacementMarker(() -> {
                    extendSlides(400, 0.9);
                })
                .lineToLinearHeading(new Pose2d(35, 9.5, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        traj13 = drive.trajectoryBuilder(traj12.end(), false)
                .addDisplacementMarker(() -> {
                    extendSlides(1000, 0.9);
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(39, 9.4, Math.toRadians(-140)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(100), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajdropConeLol = drive.trajectoryBuilder(traj13.end(), false)
                .forward(3)
                .addDisplacementMarker(() -> {
                    claw.setPosition(0);
                })
                .build();

        traj14 = drive.trajectoryBuilder(trajdropConeLol.end(), false)
                .lineToLinearHeading(new Pose2d(43.1, 9.51, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(100), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    retractSlides(0.9);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
                })
                .build();


        traj15 = drive.trajectoryBuilder(traj14.end(), false)
                .addDisplacementMarker(() -> {
                    retractSlides(0.6);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
                })
                .forward(0.1)
                .build();

        // Beginning of 3rd cycle
        /*
        traj16 = drive.trajectoryBuilder(traj15.end(), false)
                .lineToLinearHeading(new Pose2d(53, 9.5, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    claw.setPosition(0);

                })
                .build();

        traj17 = drive.trajectoryBuilder(traj16.end(), false)
                .addDisplacementMarker(() -> {
                    LeftServo.setPosition(0.9);
                    RightServo.setPosition(0.9);
                    extendSlides(80, 0.9);
                })
                .forward(8, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        traj18 = drive.trajectoryBuilder(traj17.end(), false)
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                })
                .back(0.1, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        traj19 = drive.trajectoryBuilder(traj18.end(), false)
                .addDisplacementMarker(() -> {
                    extendSlides(400, 0.9);
                })
                .lineToLinearHeading(new Pose2d(35, 9.5, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        traj20 = drive.trajectoryBuilder(traj19.end(), false)
                .addDisplacementMarker(() -> {
                    extendSlides(1030, 0.85);
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);
                })
                .lineToLinearHeading(new Pose2d(39, 9.4, Math.toRadians(-140)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(100), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    claw.setPosition(0);
                })
                .build();

        traj21 = drive.trajectoryBuilder(traj20.end(), false)
                .lineToLinearHeading(new Pose2d(43.1, 9.51, Math.toRadians(0)), SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(100), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    retractSlides(0.3);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
                })
                .build();

        traj22 = drive.trajectoryBuilder(traj21.end(), false)
                .addDisplacementMarker(() -> {
                    retractSlides(0.3);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
                })
                .forward(0.1)
                .build();


         */

        traj1Right = drive.trajectoryBuilder(traj15.end(), false)
                .back(30)
                .build();

        traj1Middle = drive.trajectoryBuilder(traj15.end(), false)
                .addDisplacementMarker(() -> {
                    retractSlides(0.8);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
                })
                .back(4)
                .build();

        traj1Left = drive.trajectoryBuilder(traj15.end(), false)
                .forward(20)
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
                drive.followTrajectory(traj1);
                drive.followTrajectory(traj2);
                //drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                drive.followTrajectory(traj5);

                sleep(1000);

                drive.followTrajectory(traj6);
                drive.followTrajectory(traj7);
                drive.followTrajectory(traj8);

                // 2nd cycle

                drive.followTrajectory(traj9);
                drive.followTrajectory(traj10);
                drive.followTrajectory(traj11);

                sleep(300);

                drive.followTrajectory(traj12);
                drive.followTrajectory(traj13);

                sleep(1000);

                drive.followTrajectory(trajdropConeLol);

                sleep(1000);

                drive.followTrajectory(traj14);
                drive.followTrajectory(traj15);

                drive.followTrajectory(traj1Right);


            }else if(tagOfInterest.id == LEFT) {
                drive.followTrajectory(traj1);
                drive.followTrajectory(traj2);
                //drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                drive.followTrajectory(traj5);

                sleep(1000);

                drive.followTrajectory(traj6);
                drive.followTrajectory(traj7);
                drive.followTrajectory(traj8);

                // 2nd cycle

                drive.followTrajectory(traj9);
                drive.followTrajectory(traj10);
                drive.followTrajectory(traj11);

                sleep(300);

                drive.followTrajectory(traj12);
                drive.followTrajectory(traj13);

                sleep(1000);

                drive.followTrajectory(trajdropConeLol);

                sleep(1000);

                drive.followTrajectory(traj14);
                drive.followTrajectory(traj15);

                drive.followTrajectory(traj1Left);



            } else {
                drive.followTrajectory(traj1);
                drive.followTrajectory(traj2);
                //drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                drive.followTrajectory(traj5);

                sleep(1000);

                drive.followTrajectory(traj6);
                drive.followTrajectory(traj7);
                drive.followTrajectory(traj8);

                // 2nd cycle

                drive.followTrajectory(traj9);
                drive.followTrajectory(traj10);
                drive.followTrajectory(traj11);

                sleep(300);

                drive.followTrajectory(traj12);
                drive.followTrajectory(traj13);

                sleep(1000);
                
                drive.followTrajectory(trajdropConeLol);

                sleep(1000);

                drive.followTrajectory(traj14);
                drive.followTrajectory(traj15);

                // 3rd cycle
/*
                drive.followTrajectory(traj16);
                drive.followTrajectory(traj17);
                drive.followTrajectory(traj18);

                sleep(300);

                drive.followTrajectory(traj19);
                drive.followTrajectory(traj20);

                sleep(1000);

                drive.followTrajectory(traj21);
                drive.followTrajectory(traj22);

*/

                //drive.followTrajectory(traj1Middle);
                //drive.followTrajectory(traj2Middle);

                drive.followTrajectory(traj1Middle);
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

    public void extendSlides(int target, double upSpeed) {
        LeftSlide.setTargetPosition(target);
        RightSlide.setTargetPosition(target);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LeftSlide.setPower(upSpeed);
        RightSlide.setPower(upSpeed);
    }

    public void retractSlides( double downSpeed) {
        LeftSlide.setTargetPosition(0);
        RightSlide.setTargetPosition(0);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LeftSlide.setPower(downSpeed);
        RightSlide.setPower(downSpeed);
    }
}
