package org.firstinspires.ftc.teamcode.drive.auto.oldautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.Disabled;

import org.firstinspires.ftc.teamcode.drive.opmodes.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmodes.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmodes.SampleMecanumDrive;

@Disabled
@Autonomous(group="Blue")
public class OldTestAuto extends OpMode
{
    private DcMotorEx LeftSlide = null;
    private DcMotorEx RightSlide = null;

    private Servo LeftServo = null;
    private Servo RightServo = null;

    private Servo claw = null;

    SampleMecanumDrive drive = null;

    int currentPosition = 0;

    Trajectory traj1ParkRight, traj2ParkRight, traj1ParkMiddle, traj2ParkMiddle, traj1ParkLeft, traj2ParkLeft;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RightSlide");

        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");

        claw = hardwareMap.get(Servo.class, "claw");

        drive = new SampleMecanumDrive(hardwareMap);

        RightSlide.setDirection(DcMotorEx.Direction.REVERSE);
        RightServo.setDirection(Servo.Direction.REVERSE);

        LeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LeftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d(35, 62, Math.toRadians(90.00));
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


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        drive.followTrajectory(traj1ParkRight);
        drive.followTrajectory(traj2ParkRight);



    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        LeftServo.setPosition(0);
        RightServo.setPosition(0.625);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public void extendSlides(int target, double upSpeed) {
        LeftSlide.setTargetPosition(target);
        RightSlide.setTargetPosition(target);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LeftSlide.setPower(upSpeed);
        RightSlide.setPower(upSpeed);
    }

    public void retractSlides(int target, double downSpeed) {
        LeftSlide.setTargetPosition(0);
        RightSlide.setTargetPosition(0);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LeftSlide.setPower(downSpeed);
        RightSlide.setPower(downSpeed);
    }

}