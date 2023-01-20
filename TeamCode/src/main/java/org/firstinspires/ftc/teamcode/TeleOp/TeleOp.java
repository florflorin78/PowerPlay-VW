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
    private DcMotor Lift = null;

    Servo ServoStanga= null;
    Servo ServoDreapta= null;
    boolean GhearaB= false;
    double ValStanga=0.23;
    double ValDreapta=0.33;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");



        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        ServoStanga.setDirection(Servo.Direction.FORWARD);
        ServoDreapta.setDirection(Servo.Direction.REVERSE);

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        Lift.setPower(0);

        ServoStanga.setPosition(ValStanga);
        ServoDreapta.setPosition(ValDreapta);

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            if (ValStanga == 0.35 && ValDreapta == 0.45) {
                ValStanga = 0.23;
                ValDreapta = 0.33;
            }
            else
            { ValStanga = 0.35;
                ValDreapta = 0.45;}
            GhearaB = false;
            ServoStanga.setPosition(ValStanga);
            ServoDreapta.setPosition(ValDreapta);
        }

//        if(gamepad2.x==true && gamepad2.left_bumper==true){
//            LiftStanga.setPower(1);
////            LiftDreapta.setPower(1);
//        }
//        else if(gamepad2.b==true && gamepad2.left_bumper==true){
//            LiftStanga.setPower(-0.5);
////            LiftDreapta.setPower(-0.5);
//        }

         if(gamepad2.x==true)
            Lift.setPower(1);

        else if(gamepad2.b==true)
            Lift.setPower(-0.6);

        else
        Lift.setPower(0);






}

    @Override
    public void stop() {

    }
}


