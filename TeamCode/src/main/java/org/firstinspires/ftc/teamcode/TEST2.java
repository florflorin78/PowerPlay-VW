package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.List;

@Autonomous(name="Autonomous Program")
public class TEST2 extends LinearOpMode {

    // Declare motors and servos
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;
    private Servo clawServo;

    // Declare Vuforia
    private VuforiaLocalizer vuforia;

    // Declare junctions
    private List<Junction> junctions;

    // Declare elapsed time variable
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motor_front_left");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor_front_right");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motor_back_left");
        motorBackRight = hardwareMap.get(DcMotor.class, "motor_back_right");
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift_motor_1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift_motor_2");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        // Initialize Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "YourVuforiaKeyGoesHere";
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Initialize junctions
        junctions = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        for (Junction junction : junctions) {
            junction.setName("Junction " + junction.getName());
            junction.setLocation(createMatrix(0, 0, 0, 0, 0, 0));
            telemetry.addData(junction.getName(), junction.getLocation());
        }

        // Wait for start
        // Start autonomous routine
        runtime.reset();
        while (opModeIsActive()) {
            // Loop through junctions and find the closest one
            Junction closestJunction = null;
            double minDistance = Double.MAX_VALUE;
            for (Junction junction : junctions) {
                double distance = getDistanceToJunction(junction);
                if (distance < minDistance) {
                    closestJunction = junction;
                    minDistance = distance;
                }
            }

            // Go to the closest junction and place a cone
            goToJunction(closestJunction);
            clawServo.setPosition(1);
            sleep(1000);
            clawServo.setPosition(0);

            // Go to the starting position
            goToStartingPosition();
        }
    }

    private double getDistanceToJunction(Junction junction) {
        // Calculate the distance to the junction using the robot's location and the junction's location
        // Return the distance
        return 0; // Placeholder value, replace with actual implementation
    }


    private void driveDistance(double distance) {
        // Reset encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target position for encoders
        motorFrontLeft.setTargetPosition((int) (distance * 1120)); // 1120 encoder ticks per revolution, 4 inch wheels
        motorFrontRight.setTargetPosition((int) (distance * 1120));
        motorBackLeft.setTargetPosition((int) (distance * 1120));
        motorBackRight.setTargetPosition((int) (distance * 1120));

        // Set motors to run to position
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to motors
        motorFrontLeft.setPower(0.5);
        motorFrontRight.setPower(-0.5);
        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(-0.5);

// Wait for motors to finish
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()) {
            // Do nothing
        }

// Stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

// Set motors back to normal run mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void turnTo(double angle) {
// Reset encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target position for encoders
        motorFrontLeft.setTargetPosition((int)(angle * 1120)); // 1120 encoder ticks per revolution, 4 inch wheels
        motorFrontRight.setTargetPosition((int)(-angle * 1120));
        motorBackLeft.setTargetPosition((int)(angle * 1120));
        motorBackRight.setTargetPosition((int)(-angle * 1120));

// Set motors to run to position
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// Set power to motors
        motorFrontLeft.setPower(0.5);
        motorFrontRight.setPower(0.5);
        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(0.5);

// Wait for motors to finish
        while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()) {
// Do nothing
        }

// Stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

// Set motors back to normal run mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void pickUpCone() {
// Lower the lift
        liftMotor1.setPower(-0.5);
        liftMotor2.setPower(-0.5);
        sleep(1000);
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);

// Open the claw
        clawServo.setPosition(0.5);

// Drive forward to pick up the cone
        motorFrontLeft.setPower(0.5);
        motorFrontRight.setPower(-0.5);
        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(-0.5);
        sleep(500);

        private void pickUpCone() {
// Lower the lift
            liftMotor1.setPower(-0.5);
            liftMotor2.setPower(-0.5);
            sleep(1000);
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);

// Open the claw
            clawServo.setPosition(0.5);

// Drive forward to pick up the cone
            motorFrontLeft.setPower(0.5);
            motorFrontRight.setPower(-0.5);
            motorBackLeft.setPower(0.5);
            motorBackRight.setPower(-0.5);
            sleep(500);

// Close the claw
            clawServo.setPosition(0);

            // Drive backwards to place the cone
            motorFrontLeft.setPower(-0.5);
            motorFrontRight.setPower(0.5);
            motorBackLeft.setPower(-0.5);
            motorBackRight.setPower(0.5);
            sleep(500);

// Raise the lift
            liftMotor1.setPower(0.5);
            liftMotor2.setPower(0.5);
            sleep(1000);
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
        }

        private void driveDistance(double distance) {
// Reset encoders
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target position for encoders
            motorFrontLeft.setTargetPosition((int)(distance * 1120)); // 1120 encoder ticks per revolution, 4 inch wheels
            motorFrontRight.setTargetPosition((int)(distance * 1120));
            motorBackLeft.setTargetPosition((int)(distance * 1120));
            motorBackRight.setTargetPosition((int)(distance * 1120));

// Set motors to run to position
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// Set power to motors
            motorFrontLeft.setPower(0.5);
            motorFrontRight.setPower(-0.5);
            motorBackLeft.setPower(0.5);
            motorBackRight.setPower(-0.5);

            // Wait for motors to finish
            while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()) {
// Do nothing
            }

// Stop motors
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

// Set motors back to normal run mode
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        private void turnTo(double angle) {
// Reset encoders
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// Calculate the number of encoder ticks needed to turn the desired angle
            double ticks = angle / 360 * 1120; // 1120 encoder ticks per revolution, 4 inch wheels

            // Set target position for encoders
            motorFrontLeft.setTargetPosition((int)ticks);
            motorFrontRight.setTargetPosition((int)(-ticks));
            motorBackLeft.setTargetPosition((int)ticks);
            motorBackRight.setTargetPosition((int)(-ticks));

// Set motors to run to position
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power to motors
            motorFrontLeft.setPower(0.5);
            motorFrontRight.setPower(0.5);
            motorBackLeft.setPower(0.5);
            motorBackRight.setPower(0.5);

// Wait for motors to finish
            while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()) {
// Do nothing
            }

// Stop motors
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // Set motors back to normal run mode
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        createMatrix(float x, float y, float z, float u, float v, float w) {
            org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Matrix44F matrix = new org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Matrix44F();
            matrix.data[0] = x;
            matrix.data[1] = y;
            matrix.data[2] = z;
            matrix.data[3] = 0;
            matrix.data[4] = u;
            matrix.data[5] = v;
            matrix.data[6] = w;
            matrix.data[7] = 0;
            matrix.data[8] = 0;
            matrix.data[9] = 0;
            matrix.data[10] = 0;
            matrix.data[11] = 0;
            matrix.data[12] = 0;
            matrix.data[13] = 0;
            matrix.data[14] = 0;
            matrix.data[15] = 1;
            return matrix;
        }
        private void goToJunction(Junction junction) {
// Calculate the distance and angle needed to reach the junction
            double distance = getDistanceToJunction(junction);
            double angle = getAngleToJunction(junction);

// Turn to the desired angle
            turnTo(angle);

// Move the desired distance
            moveDistance(distance);
        }
        private void goToStartingPosition() {
// Calculate the distance and angle needed to reach the starting position
            double distance = getDistanceToStartingPosition();
            double angle = getAngleToStartingPosition();

// Turn to the desired angle
            turnTo(angle);

// Move the desired distance
            moveDistance(distance);
        }

        private void moveDistance(double distance) {
// Reset encoders
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// Calculate the number of encoder ticks needed to move the desired distance
            double ticks = distance / (Math.PI * 4) * 1120; // 1120 encoder ticks per revolution, 4 inch wheels
            // Set target position for encoders
            motorFrontLeft.setTargetPosition((int)ticks);
            motorFrontRight.setTargetPosition((int)ticks);
            motorBackLeft.setTargetPosition((int)ticks);
            motorBackRight.setTargetPosition((int)ticks);

// Set motors to run to position
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// Set power to motors
            motorFrontLeft.setPower(0.5);
            motorFrontRight.setPower(0.5);
            motorBackLeft.setPower(0.5);
            motorBackRight.setPower(0.5);

// Wait for motors to finish
            while (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()) {
// Do nothing
            }

// Stop motors
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

// Set motors back to normal run mode
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        private double getDistanceToJunction(Junction junction) {
            double x1 = junction.getLocation().get(0);
            double y1 = junction.getLocation().get(1);
            double z1 = junction.getLocation().get(2);
            double x2 = robot.getLocation().get(0);
            double y2 = robot.getLocation().get(1);
            double z2 = robot.getLocation().get(2);
            return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2) + Math.pow(z2 - z1, 2));
        }