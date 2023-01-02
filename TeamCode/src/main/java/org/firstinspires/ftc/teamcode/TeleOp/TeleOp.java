package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.camera.CAM;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LiftStanga = null;
    private DcMotor LiftDreapta = null;

    Servo ServoStanga= null;
    Servo ServoDreapta= null;
    boolean GhearaB= false;
    double ValStanga=0.25;
    double ValDreapta=0.35;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LiftStanga = hardwareMap.get(DcMotor.class, "LiftStanga");
        LiftDreapta = hardwareMap.get(DcMotor.class, "LiftDreapta");
        ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");


        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LiftStanga.setDirection(DcMotor.Direction.FORWARD);
        LiftDreapta.setDirection(DcMotor.Direction.REVERSE);
        ServoStanga.setDirection(Servo.Direction.FORWARD);
        ServoDreapta.setDirection(Servo.Direction.REVERSE);

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
        LiftStanga.setPower(0);
        LiftDreapta.setPower(0);
        ServoStanga.setPosition(ValStanga);
        ServoDreapta.setPosition(ValDreapta);

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    /*LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}
    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);
        if(gamepad1.right_bumper == true){ denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 3.5);}
        double LeftFrontPower = (y - x + rx)/denominator;
        double LeftBackPower = (y + x + rx)/denominator;
        double RightFrontPower = (y + x - rx)/denominator;
        double RightBackPower = (y - x - rx)/denominator;

        LeftFront.setPower(LeftFrontPower);   // +
        LeftBack.setPower(LeftBackPower);     // -
        RightFront.setPower(RightFrontPower); // -
        RightBack.setPower(RightBackPower);   // +

        if(gamepad2.right_bumper == false && GhearaB == false) GhearaB = true;
        if(gamepad2.right_bumper == true && GhearaB == true) {
            if (ValStanga == 0.05 && ValDreapta == 0.25) {
                ValStanga = 0.25;
                ValDreapta = 0.35;
            }
            else
            { ValStanga = 0.05;
                ValDreapta = 0.25;}
            GhearaB = false;
            ServoStanga.setPosition(ValStanga);
            ServoDreapta.setPosition(ValDreapta);
        }

        if(gamepad2.x==true && gamepad2.left_bumper==true){
            LiftStanga.setPower(0.88);
            LiftDreapta.setPower(0.88);}
        else if(gamepad2.b==true && gamepad2.left_bumper==true){
            LiftStanga.setPower(-0.65);
            LiftDreapta.setPower(-0.65);}
        else if(gamepad2.x==true){
            LiftStanga.setPower(0.4);
            LiftDreapta.setPower(0.4);}
        else if(gamepad2.b==true){
            LiftStanga.setPower(-0.2);
            LiftDreapta.setPower(-0.2);}
        else
        {LiftStanga.setPower(0);
         LiftDreapta.setPower(0);}

    }
    @Override
    public void stop() {}
}