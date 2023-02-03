package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Advanced.PoseStorage;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOpLocatie")
public class TeleOpNou extends LinearOpMode {
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.left_stick_x
                                    -gamepad1.left_stick_y
                    )
            );

            drive.update();

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            robot.setDrivePower(x, y, rx);

            if(!gamepad2.right_bumper && !robot.GhearaB) robot.GhearaB = true;
            if(gamepad2.right_bumper && robot.GhearaB) {
                if ( robot.GhearaValStanga ==  robot.closedStanga &&  robot.GhearaValDreapta ==  robot.closedDreapta) {
                    robot.GhearaValStanga =  robot.openStanga;
                    robot.GhearaValDreapta =  robot.openDreapta;
                }
                else{
                    robot.GhearaValStanga =  robot.closedStanga;
                    robot.GhearaValDreapta =  robot.closedDreapta;}
                robot.GhearaB = false;

                robot.ServoStanga.setPosition( robot.GhearaValStanga);
                robot.ServoDreapta.setPosition( robot.GhearaValDreapta);
                //GhearaVal =
            }

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();


        }
    }

    class Robot {
        private DcMotorEx leftFront, leftRear, rightRear, rightFront, LiftDreapta, LiftStanga;
        private Servo ServoStanga, ServoDreapta;

        boolean GhearaB = false;
        double closedStanga = 0.53,openStanga = 0.37, closedDreapta = 0.53, openDreapta = 0.37, GhearaValStanga = 0.53, GhearaValDreapta = 0.53;

        public Robot(HardwareMap hardwareMap) {
            // Initialize motors
            leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
            rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");

            LiftStanga = hardwareMap.get(DcMotorEx.class, "LiftStanga");
            LiftDreapta = hardwareMap.get(DcMotorEx.class, "LiftDreapta");

            ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
            ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);

            LiftStanga.setDirection(DcMotor.Direction.FORWARD);
            LiftDreapta.setDirection(DcMotor.Direction.REVERSE);

            ServoStanga.setDirection(Servo.Direction.REVERSE);
            ServoDreapta.setDirection(Servo.Direction.FORWARD);

            LiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        public void setDrivePower(double x, double y, double rx) {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);
            if(gamepad1.left_bumper == true){ denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 4);}
            if(gamepad1.right_bumper == true) { denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.5);}
            double LeftFrontPower = (y - x + rx)/denominator;
            double LeftBackPower = (y + x + rx)/denominator;
            double RightFrontPower = (y + x - rx)/denominator;
            double RightBackPower = (y - x - rx)/denominator;

            leftFront.setPower(LeftFrontPower);   // +
            leftRear.setPower(LeftBackPower);     // -
            rightFront.setPower(RightFrontPower); // -
            rightRear.setPower(RightBackPower);   // +

        }
    }
}