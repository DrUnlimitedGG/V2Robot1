package org.firstinspires.ftc.teamcode.drive.auto.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmodes.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmodes.SampleMecanumDrive;

@Autonomous(group="Blue")
public class BlueLeftParkRight extends OpMode
{
    private DcMotorEx LeftSlide = null;
    private DcMotorEx RightSlide = null;

    private Servo LeftServo = null;
    private Servo RightServo = null;

    private Servo claw = null;

    SampleMecanumDrive drive = null;

    int targetPosition = 0;

    Trajectory traj1, traj2, traj3, traj4, traj5;


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

        Pose2d startPose = new Pose2d(35, 60.5, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);

         traj1 = drive.trajectoryBuilder(startPose, false)
                 .addDisplacementMarker(() -> {
                     claw.setPosition(0.3);
                 })
                .lineToLinearHeading(new Pose2d(12.13, 50, Math.toRadians(90)))
                 .addDisplacementMarker(() -> {
                    LeftServo.setPosition(1);
                    RightServo.setPosition(1);
                 })
                .build();

        traj2 = drive.trajectoryBuilder(traj1.end(), false)
                .lineToLinearHeading(new Pose2d(12.13, 9.79, Math.toRadians(-91)))
                .build();

        traj3 = drive.trajectoryBuilder(traj2.end(), false)
                .lineToLinearHeading(new Pose2d(19, 13, Math.toRadians(-50)))
                .addDisplacementMarker(() -> {
                    targetPosition = 1150;

                    LeftSlide.setTargetPosition(targetPosition);
                    RightSlide.setTargetPosition(targetPosition);

                    LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                    LeftSlide.setPower(0.6);
                    RightSlide.setPower(0.6);
                })
                .build();


        traj4 = drive.trajectoryBuilder(traj3.end(), false)
                .lineToLinearHeading(new Pose2d(21, 10, Math.toRadians(-50)))
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
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        //drive.followTrajectory(traj4);


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

}