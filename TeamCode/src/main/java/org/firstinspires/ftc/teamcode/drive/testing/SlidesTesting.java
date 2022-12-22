package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(group="Testing")
public class SlidesTesting extends OpMode {

    // Slide motors
    private DcMotorEx LeftSlide = null;
    private DcMotorEx RightSlide = null;

    // Target position
    public static double targetPosition = 0;

    @Override
    public void init() {
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RightSlide");

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LeftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightSlide.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        LeftSlide.setTargetPosition(0);
        RightSlide.setTargetPosition(0);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {


    }

}
