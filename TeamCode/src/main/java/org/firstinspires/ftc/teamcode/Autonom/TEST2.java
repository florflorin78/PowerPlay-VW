package org.firstinspires.ftc.teamcode.Autonom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Autonomous(name="Autonom cu FTC Dashboard", group="Exemplu")
public class TEST2 extends LinearOpMode {

    // Declarăm motoarele şi servo-urile robotului
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor liftMotor;
    private Servo clawServo1;
    private Servo clawServo2;

    // Declarăm variabilele pentru poziţia şi unghiul robotului
    private double xPos = 0.0;
    private double yPos = 0.0;
    private double angle = 0.0;

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
            } else if (xSpeed < -targetSpeed) {
                xSpeed = -targetSpeed;
            }
            if (ySpeed > targetSpeed) {
                ySpeed = targetSpeed;
            } else if (ySpeed < -targetSpeed) {
                ySpeed = -targetSpeed;
            }
            if (angSpeed > targetSpeed) {
                angSpeed = targetSpeed;
            } else if (angSpeed < -targetSpeed) {
                angSpeed = -targetSpeed;
            }

// setăm vitezele motoarelor în funcţie de poziţia şi viteza dorite
            motorFrontLeft.setPower(xSpeed + ySpeed + angSpeed);
            motorFrontRight.setPower(-xSpeed + ySpeed - angSpeed);
            motorBackLeft.setPower(xSpeed - ySpeed + angSpeed);
            motorBackRight.setPower(-xSpeed - ySpeed - angSpeed);

// actualizăm poziţia şi unghiul robotului folosind vitezele motoarelor
            updatePosition();

// Aşteptăm o perioadă de timp pentru a nu încărca prea mult procesorul
            Thread.sleep(50);

// Verificăm dacă am ajuns la junction-ul dorit
            if (Math.abs(positionError) < positionTolerance && Math.abs(angleError) < angleTolerance) {
// Dacă da, atunci ne oprim
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);

                // Aşteptăm ca lift-ul să se ridice până la poziţia dorită
                liftMotor.setPower(0.5);
                while (opModeIsActive() && liftMotor.getCurrentPosition() < dashboard.getNumber("Lift Target Position", 0)) {
                    telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
                    telemetry.update();
                    Thread.sleep(100);
                }
                liftMotor.setPower(0);

                // Deschidem gheara şi luăm conul
                clawServo1.setPosition(dashboard.getNumber("Claw Servo 1 Open Position", 0.0));
                clawServo2.setPosition(dashboard.getNumber("Claw Servo 2 Open Position", 0.0));
                Thread.sleep(1000);
                clawServo1.setPosition(dashboard.getNumber("Claw Servo 1 Closed Position", 0.0));
                clawServo2.setPosition(dashboard.getNumber("Claw Servo 2 Closed Position", 0.0));

                // Ne întoarcem la poziţia de start
                targetDistance = 0.0;
                angle = 0.0;
            } else {
                // actualizăm poziţia şi unghiul robotului folosind vitezele calculate
                xPos += xSpeed * 0.1;
                yPos += ySpeed * 0.1;
                angle += angSpeed * 0.1;

                // setăm vitezele motoarelor în funcţie de poziţia şi viteza dorite
                motorFrontLeft.setPower(xSpeed + ySpeed + angSpeed);
                motorFrontRight.setPower(-xSpeed + ySpeed - angSpeed);
                motorBackLeft.setPower(xSpeed - ySpeed + angSpeed);
                motorBackRight.setPower(-xSpeed - ySpeed - angSpeed);

                // actualizăm poziţia şi unghiul robotului folosind vitezele motoarelor
                updatePosition(xSpeed, ySpeed, angSpeed);

// Verificăm dacă am ajuns la junction-ul cel mai apropiat
                if (Math.abs(positionError) < positionTolerance && Math.abs(angleError) < angleTolerance) {
                    // Dacă da, atunci ne oprim
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    break;
                }

// Aşteptăm pentru a da timp sistemului de control să se actualizeze
                sleep(50);
            }

            // Ridicăm conul folosind lift-ul
            liftMotor.setPower(0.5);
            sleep(500);
            liftMotor.setPower(0.0);

            // Deschidem ghearele şi luăm conul
            clawServo1.setPosition(0.5);
            clawServo2.setPosition(0.5);
            sleep(500);

// Întoarcem robotul spre cazul detectat
            moveRobot(0.0, 0.0, -angle);

// Mergem către cazul detectat
            moveRobot(xPos, yPos, 0.0);

// Întoarcem robotul spre caz
            moveRobot(0.0, 0.0, -angle);

// Aşezăm conul în caz
            clawServo1.setPosition(1.0);
            clawServo2.setPosition(1.0);
            sleep(500);

// Întoarcem robotul spre poziţia de start
            moveRobot(-xPos, -yPos, 0.0);

            // Parchează robotul în zona specificată
            moveRobot(0.0, dashboard.getNumber("Parking Distance", 0.0), 0.0);

            // Oprim motoarele
            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);
            liftMotor.setPower(0.0);

// Aşteptăm ca jocul să se termine
            while (opModeIsActive()) {
                idle();
            }
        }

// Funcţia de mutare a robotului cu un anumit număr de centimetri pe x şi y şi un anumit unghi în grade
        private void moveRobot(double x, double y, double angle) {
// Convertim distanţa dorită în număr de ture de roată
            int xDistance = (int) (x * ENCODER_TICKS_PER_CM);
            int yDistance = (int) (y * ENCODER_TICKS_PER_CM);

// Setăm motoarele pentru a merge cu encodere
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// Setăm poziţiile pentru motoare
            motorFrontLeft.setTargetPosition(xDistance);
            motorFrontRight.setTargetPosition(yDistance);
            motorBackLeft.setTargetPosition(-xDistance);
            motorBackRight.setTargetPosition(-yDistance);

// Pornim motoarele
            motorFrontLeft.setPower(0.5);
            motorFrontRight.setPower(0.5);
            motorBackLeft.setPower(0.5);
            motorBackRight.setPower(0.5);

// Aşteptăm până când motoarele au ajuns la poziţia dorită
            while (opModeIsActive() && (motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy())) {
// Afişăm poziţia curentă a motoarelor
                telemetry.addData("Front Left", motorFrontLeft.getCurrentPosition());
                telemetry.addData("Front Right", motorFrontRight.getCurrentPosition());
                telemetry.addData("Back Left", motorBackLeft.getCurrentPosition());
                telemetry.addData("Back Right", motorBackRight.getCurrentPosition());
                telemetry.update();
            }
            // Oprim motoarele
            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);
            // Setăm motoarele în modul normal
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }