package org.firstinspires.ftc.teamcode.drive.auto.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmodes.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmodes.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="BlueLeft", group="Blue")
public class BlueLeft extends LinearOpMode {
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
    Trajectory traj1, traj2, traj3, traj4, traj5, traj6, traj1Right, traj1Middle, traj2Middle, traj1Left, traj2Left;

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

        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);

        traj1 = drive.trajectoryBuilder(startPose, false)
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                })

                .lineToLinearHeading(new Pose2d(13, 57, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);
                })
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end(), false)
                .lineToLinearHeading(new Pose2d(13, 9.79, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    extendSlides(1050, 0.85);
                    LeftServo.setPosition(0);
                    RightServo.setPosition(0);
                })
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end(), false)
                .lineToLinearHeading(new Pose2d(19, 9, Math.toRadians(130)))
                .build();

        traj4 = drive.trajectoryBuilder(traj3.end(), false)
                .lineToLinearHeading(new Pose2d(25.5, 6.5, Math.toRadians(133)))
                .build();

        traj5 = drive.trajectoryBuilder(traj4.end(), false)
                .strafeRight(0.1)
                .addDisplacementMarker(() -> {
                   claw.setPosition(0);
                })
                .build();

        traj6 = drive.trajectoryBuilder(traj5.end(), false)
                .lineToLinearHeading(new Pose2d(13, 10.5, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(0.7);
                    RightServo.setPosition(0.7);
                    retractSlides(0.2);
                })
                .build();

        traj1Right = drive.trajectoryBuilder(traj6.end(), false)
                .lineToLinearHeading(new Pose2d(13.58, 35, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
                })
                .build();

        traj1Middle = drive.trajectoryBuilder(traj6.end(), false)
                .lineToLinearHeading(new Pose2d(40, 9.5, Math.toRadians(90)))

                .build();

        traj2Middle = drive.trajectoryBuilder(traj1Middle.end(), false)
                .lineToLinearHeading(new Pose2d(40, 35, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
                })
                .build();

        traj1Left = drive.trajectoryBuilder(traj6.end(), false)
                .lineToLinearHeading(new Pose2d(63, 9.5, Math.toRadians(90)))
                .build();

        traj2Left = drive.trajectoryBuilder(traj1Left.end(), false)
                .lineToLinearHeading(new Pose2d(63, 35, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    claw.setPosition(1);
                    LeftServo.setPosition(0.8);
                    RightServo.setPosition(0.8);
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
                drive.followTrajectory(traj1);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                drive.followTrajectory(traj5);
                drive.followTrajectory(traj6);
                drive.followTrajectory(traj1Right);


            }else if(tagOfInterest.id == LEFT) {
                drive.followTrajectory(traj1);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                drive.followTrajectory(traj5);
                drive.followTrajectory(traj6);
                drive.followTrajectory(traj1Left);
                drive.followTrajectory(traj2Left);


            } else {
                drive.followTrajectory(traj1);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                drive.followTrajectory(traj5);
                drive.followTrajectory(traj6);
                drive.followTrajectory(traj1Middle);
                drive.followTrajectory(traj2Middle);


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
