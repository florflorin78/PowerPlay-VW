package org.firstinspires.ftc.teamcode.Autonom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Autonom", group="Exemplu")
public class TEST extends LinearOpMode {

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

// Setăm viteza dorită pentru motoarele de deplasare
        double targetSpeed = 0.5;

// Setăm o toleranţă pentru eroarea de poziţionare
        double positionTolerance = 0.1;

// Setăm o toleranţă pentru eroarea de unghi
        double angleTolerance = 0.1;

// Setăm o limită de timp pentru a evita bucla infinită
        long timeLimit = System.currentTimeMillis() + 60000; // 60 seconds

// Începem bucla principală de control a robotului
        while (opModeIsActive() && System.currentTimeMillis() < timeLimit) {
// Verificăm dacă robotul a ajuns la junction-ul cel mai apropiat
            if (Math.abs(xPos) < positionTolerance && Math.abs(yPos) < positionTolerance && Math.abs(angle) < angleTolerance) {
// Dacă da, atunci ne oprim
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
// Aşteptăm până când jocul se termină
                while (opModeIsActive());
            }

/**

 Această funcţie deplasează robotul în direcţia dorită
 @param xSpeed viteza pe axa X (înainte/înapoi)
 @param ySpeed viteza pe axa Y (stânga/dreapta)
 @param angle unghiul de rotaţie
 */
            public void moveRobot(double xSpeed, double ySpeed, double angle) {
// setăm vitezele motoarelor în funcţie de poziţia şi viteza dorite
                motorFrontLeft.setPower(xSpeed + ySpeed + angle);
                motorFrontRight.setPower(-xSpeed + ySpeed - angle);
                motorBackLeft.setPower(xSpeed - ySpeed + angle);
                motorBackRight.setPower(-xSpeed - ySpeed - angle);
// actualizăm poziţia şi unghiul robotului folosind vitezele motoarelor
                xPos += xSpeed * Math.cos(angle) - ySpeed * Math.sin(angle);
                yPos += xSpeed * Math.sin(angle) + ySpeed * Math.cos(angle);
                angle += angle;
            }

/**

 Această funcţie ridică sau coboară lift-ul robotului
 @param power puterea motorului de lift
 */
            public void moveLift(double power;) {
                liftMotor.setPower(power);
            }
/**

 Această funcţie deschide sau închide gheara robotului
 @param position poziţia servo-urilor pentru gheară
 */
            public void moveClaw(double position) {
                clawServo1.setPosition(position);
                clawServo2.setPosition(position);
            }
        }
    }
}