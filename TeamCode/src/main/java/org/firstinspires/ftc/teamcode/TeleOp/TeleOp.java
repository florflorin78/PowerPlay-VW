package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Odometry;
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

     Servo ServoGheara = null;
    boolean GhearaB = false;
    double GhearaVal = 0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LiftStanga = hardwareMap.get(DcMotor.class, "LiftStanga");
        LiftDreapta = hardwareMap.get(DcMotor.class, "LiftDreapta");

        ServoGheara = hardwareMap.get(Servo.class, "ServoGheara");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LiftStanga.setDirection(DcMotor.Direction.FORWARD);
        LiftDreapta.setDirection(DcMotor.Direction.REVERSE);
        ServoGheara.setDirection(Servo.Direction.FORWARD);

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        LiftStanga.setPower(0);
        LiftDreapta.setPower(0);

        ServoGheara.setPosition(GhearaVal);

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LiftStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        if(gamepad1.left_bumper == true){ denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 4);}
        if(gamepad1.right_bumper == true) { denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.5);}
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
            if (GhearaVal == 0.6) {
                GhearaVal = -0.45;
            }
            else {
                GhearaVal = 0.6;
                GhearaB = false;
            }
            ServoGheara.setPosition(GhearaVal);
            //GhearaVal =
        }


        if(gamepad2.x==true) {
            LiftDreapta.setPower(0.8);
            LiftStanga.setPower(0.8);
        }
        else if(gamepad2.b==true) {
            LiftDreapta.setPower(-0.55);
            LiftStanga.setPower(-0.55);
        }
        else {
            LiftDreapta.setPower(0);
            LiftStanga.setPower(0);
        }


}

    @Override
    public void stop() {

    }
}


