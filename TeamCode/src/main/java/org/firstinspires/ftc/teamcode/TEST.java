package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Autonomous(name="Autonom cu FTC Dashboard", group="Exemplu")
public class TEST extends LinearOpMode {

    // Declarăm motoarele şi servo-urile robotului
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor liftMotor;
    private Servo clawServo1;
    private Servo clawServo2;

// Declarăm variabile pentru poziţia şi unghiul robotului
    private double xPos = 0.0;
    private double yPos = 0.0;
    private double angle = 0.0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    // Declarăm variabilele pentru setarea vitezelor motoarelor
    private double targetSpeed = 0.5;
    private double positionTolerance = 0.1;
    private double angleTolerance = 0.1;
    private long timeLimit = System.currentTimeMillis() + 10000;

    // Declarăm variabilele pentru setarea comenzilor din FTC Dashboard
    private String caseColor = "None";
    private double targetDistance = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Iniţializăm motoarele şi servo-urile robotului
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor_front_left");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor_front_right");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor_back_left");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor_back_right");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        clawServo1 = hardwareMap.get(Servo.class, "claw_servo_1");
        clawServo2 = hardwareMap.get(Servo.class, "claw_servo_2");

        // Aşteptăm ca jocul să înceapă
        waitForStart();

        // Setăm comenzile din FTC Dashboard
        caseColor = dashboard.getString("Case Color", "None");
        targetDistance = dashboard.getNumber("Target Distance", 0.0);

        // Începem bucla principală de control a robotului
        while (opModeIsActive() && System.currentTimeMillis() < timeLimit) {
            // Calculăm distanţa până la junction-ul cel mai apropiat
            double distance = Math.sqrt(Math.pow(xPos, 2) + Math.pow(yPos, 2));

            // Calculăm eroarea de poziţionare
            double positionError = targetDistance - distance;

            // Calculăm eroarea de unghi
            double angleError = 0.0 - angle;

            // Setăm vitezele motoarelor în funcţie de erorile calculate
            double xSpeed = positionError * 0.01;
            double ySpeed = 0.0;
            double angSpeed = angleError * 0.01;
            // limităm vitezele la valorile maxime permise
            if (xSpeed > targetSpeed) {
                xSpeed = targetSpeed;
            }
            if (ySpeed > targetSpeed) {
                ySpeed = targetSpeed;
            }
            if (angSpeed > targetSpeed) {
                angSpeed = targetSpeed;
            }
            if (xSpeed < -targetSpeed) {
                xSpeed = -targetSpeed;
            }
            if (ySpeed < -targetSpeed) {
                ySpeed = -targetSpeed;
            }
            if (angSpeed < -targetSpeed) {
                angSpeed = -targetSpeed;
            }

            // Setăm vitezele motoarelor
            motorFrontLeft.setPower(xSpeed + ySpeed + angSpeed);
            motorFrontRight.setPower(-xSpeed + ySpeed - angSpeed);
            motorBackLeft.setPower(-xSpeed + ySpeed + angSpeed);
            motorBackRight.setPower(xSpeed + ySpeed - angSpeed);

            // Verificăm dacă am ajuns la junction
            if (Math.abs(positionError) < positionTolerance && Math.abs(angleError) < angle)
            // Setăm vitezele motoarelor
            motorFrontLeft.setPower(xSpeed + ySpeed + angSpeed);
            motorFrontRight.setPower(-xSpeed + ySpeed - angSpeed);
            motorBackLeft.setPower(-xSpeed + ySpeed + angSpeed);
            motorBackRight.setPower(xSpeed + ySpeed - angSpeed);

            // Verificăm dacă am ajuns la junction
            if (Math.abs(positionError) < positionTolerance && Math.abs(angleError) < angleTolerance) {
            // Oprim motoarele
            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);

            // Întoarcem robotul spre junction
            angle = 0.0;

                // Verificăm culoarea casei
                if (caseColor.equalsIgnoreCase("Red")) {
                    // Deschidem ghearele
                    clawServo1.setPosition(0.9);
                    clawServo2.setPosition(0.1);

                    // Ridicăm braţul
                    liftMotor.setPower(-0.5);
                    sleep(3000);
                    liftMotor.setPower(0.0);

                    // Închidem ghearele
                    clawServo1.setPosition(0.1);
                    clawServo2.setPosition(0.9);

                    // Coborâm braţul
                    liftMotor.setPower(0.5);
                    sleep(3000);
                    liftMotor.setPower(0.0);
                } else if (caseColor.equalsIgnoreCase("Blue")) {
                    // Deschidem ghearele
                    clawServo1.setPosition(0.1);
                    clawServo2.setPosition(0.9);

                    // Ridicăm braţul
                    liftMotor.setPower(-0.5);
                    sleep(3000);
                    liftMotor.setPower(0.0);

                    // Închidem ghearele
                    clawServo1.setPosition(0.9);
                    clawServo2.setPosition(0.1);

                    // Coborâm braţul
                    liftMotor.setPower(0.5);
                    sleep(3000);
                    liftMotor.setPower(0.0);
                }
                // Verificăm dacă am atins ţinta
                if (Math.abs(positionError) < positionTolerance && Math.abs(angleError) < angleTolerance) {
// Oprim motoarele
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    break;
                }

// Setăm vitezele motoarelor
                motorFrontLeft.setPower(-ySpeed + xSpeed + angSpeed);
                motorFrontRight.setPower(ySpeed + xSpeed - angSpeed);
                motorBackLeft.setPower(-ySpeed - xSpeed + angSpeed);
                motorBackRight.setPower(ySpeed - xSpeed - angSpeed);

// Trimitem informaţiile către FTC Dashboard
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("xPos", xPos);
                packet.put("yPos", yPos);
                packet.put("angle", angle);
                packet.put("distance", distance);
                packet.put("positionError", positionError);
                packet.put("angleError", angleError);
                dashboard.sendTelemetryPacket(packet);

// Întrerupem execuţia pentru câteva milisecunde
                sleep(50);

                // Trimitem un mesaj către FTC Dashboard
                dashboard.sendTelemetryPacket(new TelemetryPacket().put("Misiune îndeplinită!", "Robotul a atins ţinta!"));
            }
        }
    }
