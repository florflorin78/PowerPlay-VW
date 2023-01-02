package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.camera.CAM;

public class Robot {

    public DcMotor LeftFront = null;
    public DcMotor LeftBack = null;
    public DcMotor RightFront = null;
    public DcMotor RightBack = null;
    public DcMotor LiftStanga = null;
    public DcMotor LiftDreapta = null;
    public Servo ServoStanga = null;
    public Servo ServoDreapta = null;
    boolean GhearaB = false;


    public static final double GhVal = 0.5;

    HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    public Robot() {
    }

    public void init (HardwareMap ahwMap) {

        hwMap = ahwMap;

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        LeftFront = hwMap.get(DcMotor.class, "LeftFront");
        LeftBack = hwMap.get(DcMotor.class, "LeftBack");
        RightFront = hwMap.get(DcMotor.class, "RightFront");
        RightBack = hwMap.get(DcMotor.class, "RightBack");
        LiftStanga = hwMap.get(DcMotor.class, "LiftStanga");
        LiftDreapta = hwMap.get(DcMotor.class, "LiftStanga");
        ServoStanga = hwMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hwMap.get(Servo.class, "ServoDreapta");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LiftStanga.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoStanga.setDirection(Servo.Direction.FORWARD);
        ServoDreapta.setDirection(Servo.Direction.REVERSE);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ServoStanga.setPosition(GhVal);
        ServoDreapta.setPosition(GhVal);
    }
}