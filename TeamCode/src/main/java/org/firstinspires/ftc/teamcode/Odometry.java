package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry extends LinearOpMode {
    // Declararea motorilor si a encodeurilor
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor encoderFrontRight;
    DcMotor encoderFrontLeft;
    DcMotor encoderBackRight;
    DcMotor encoderBackLeft;

    // Declararea constantelor pentru rezolutia encodeurilor si diametrul rotilor
    static final double ENCODER_RESOLUTION = 1000;
    static final double WHEEL_DIAMETER = 50;
    static final double WHEEL_BASE = 500;

    // Declararea variabilelor pentru pozitia si unghiul robotului
    double xPos = 0;
    double yPos = 0;
    double angle = 0;

    public void runOpMode() {
        // Initalizarea motorilor si a encodeurilor
        motorFrontRight = hardwareMap.dcMotor.get("front_right");
        motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        motorBackRight = hardwareMap.dcMotor.get("back_right");
        motorBackLeft = hardwareMap.dcMotor.get("back_left");
        encoderFrontRight = hardwareMap.dcMotor.get("front_right_encoder");
        encoderFrontLeft = hardwareMap.dcMotor.get("front_left_encoder");
        encoderBackRight = hardwareMap.dcMotor.get("back_right_encoder");
        encoderBackLeft = hardwareMap.dcMotor.get("back_left_encoder");

        // Setarea modului de operare al encodeurilor la RUN_USING_ENCODER
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Citirea numarului de impulsuri de la encodeuri
            int frontRightCount = encoderFrontRight.getCurrentPosition();
            int frontLeftCount = encoderFrontLeft.getCurrentPosition();
            int backRightCount = encoderBackRight.getCurrentPosition();
            int backLeftCount = encoderBackLeft.getCurrentPosition();

            // Calcularea rotatiei rotilor in radiani
            double frontRightRot = frontRightCount * 2 * Math.PI / ENCODER_RESOLUTION;
            double frontLeftRot = frontLeftCount * 2 * Math.PI / ENCODER_RESOLUTION;
            double backRightRot = backRightCount * 2 * Math.PI / ENCODER_RESOLUTION;
            double backLeftRot = backLeftCount * 2 * Math.PI / ENCODER_RESOLUTION;

            // Calcularea vitezei rotilor in m/s
            double frontRightVel = frontRightRot * WHEEL_DIAMETER / 2;
            double frontLeftVel = frontLeftRot * WHEEL_DIAMETER / 2;
            double backRightVel = backRightRot * WHEEL_DIAMETER / 2;
            double backLeftVel = backLeftRot * WHEEL_DIAMETER / 2;

            // Calcularea translatiei in m
            double dx = (frontRightVel + frontLeftVel + backRightVel + backLeftVel) / 4;
            double dy = (frontRightVel + backRightVel - frontLeftVel - backLeftVel) / (4 * Math.tan(Math.PI / 4));

            // Actualizarea pozitiei robotului
            xPos += dx * Math.cos(angle) - dy * Math.sin(angle);
            yPos += dx * Math.sin(angle) + dy * Math.cos(angle);

            // Calcularea vitezei angulare in rad/s
            double dTheta = (frontRightVel - backRightVel + backLeftVel - frontLeftVel) / (4 * WHEEL_BASE);

            // Actualizarea unghiului robotului
            angle += dTheta;

            // Afisarea pozitiei si unghiului robotului pe telemetry
            telemetry.addData("xPos", xPos);
            telemetry.addData("yPos", yPos);
            telemetry.addData("angle", angle);
            telemetry.update();
        }
    }
}