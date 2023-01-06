package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.List;

@Autonomous(name="Autonomous Program")
public class TEST1 extends LinearOpMode {

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
    }

    private void goToJunction(Junction junction) {
        // Go to the junction using the robot's location and the junction's location
    }

    private void goToStartingPosition() {
        // Go to the starting position using the robot's location
    }
    private double getDistanceToJunction(Junction junction) {
        // Calculate the distance to the junction using the robot's location and the junction's location
        // Return the distance
        double x = junction.getLocation().get(0);
        double y = junction.getLocation().get(1);
        double z = junction.getLocation().get(2);
        double robotX = getRobotLocation().get(0);
        double robotY = getRobotLocation().get(1);
        double robotZ = getRobotLocation().get(2);
        return Math.sqrt(Math.pow(x - robotX, 2) + Math.pow(y - robotY, 2) + Math.pow(z - robotZ, 2));
    }

    private void goToJunction(Junction junction) {
        // Go to the junction using the robot's location and the junction's location
        double x = junction.getLocation().get(0);
        double y = junction.getLocation().get(1);
        double z = junction.getLocation().get(2);
        double robotX = getRobotLocation().get(0);
        double robotY = getRobotLocation().get(1);
        double robotZ = getRobotLocation().get(2);

        // Calculate the difference in x, y, and z between the robot's location and the junction's location
        double deltaX = x - robotX;
        double deltaY = y - robotY;
        double deltaZ = z - robotZ;

        // Calculate the rotation needed to face the junction
        double angle = Math.atan2(deltaY, deltaX);

        // Turn to face the junction
        turnToAngle(angle);

        // Drive to the junction
        driveDistance(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
    }
}
    private double getDistanceToJunction(Junction junction) {
        // Calculate the distance to the junction using the robot's location and the junction's location
        // Return the distance
        double x = junction.getLocation().get(0);
        double y = junction.getLocation().get(1);
        double z = junction.getLocation().get(2);
        double robotX = getRobotLocation().get(0);
        double robotY = getRobotLocation().get(1);
        double robotZ = getRobotLocation().get(2);
        return Math.sqrt(Math.pow(x - robotX, 2) + Math.pow(y - robotY, 2) + Math.pow(z - robotZ, 2));
    }

    private void goToJunction(Junction junction) {
        // Go to the junction using the robot's location and the junction's location
        double x = junction.getLocation().get(0);
        double y = junction.getLocation().get(1);
        double z = junction.getLocation().get(2);
        double robotX = getRobotLocation().get(0);
        double robotY = getRobotLocation().get(1);
        double robotZ = getRobotLocation().get(2);

        // Calculate the difference in x, y, and z between the robot's location and the junction's location
        double deltaX = x - robotX;
        double deltaY = y - robotY;
        double deltaZ = z - robotZ;

        // Calculate the rotation needed to face the junction
        double angle = Math.atan2(deltaY, deltaX);

        // Turn to face the junction
        turnToAngle(angle);

        // Drive to the junction
        driveDistance(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
    }

    private void goToStartingPosition() {
        // Go to the starting position using the robot's location
        double x = 0;
        double y = 0;
        double z = 0;
        double robotX = getRobotLocation().get(0);
        double robotY = getRobotLocation().get(1);
        double robotZ = getRobotLocation().get(2);

        // Calculate the difference in x, y, and z between the robot's location and the starting position
        double deltaX = x - robotX;
        double deltaY = y - robotY;
        double deltaZ = z - robotZ;

        // Calculate the rotation needed to face the starting position
        double angle = Math.atan2(deltaY, deltaX);

        // Turn to face the starting position
        turnToAngle(angle);

        // Drive to the starting position
        driveDistance(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
    }
    private void pickUpCone() {
// Lower the lift
        liftMotor1.setPower(-0.5);
        liftMotor2.setPower(-0.5);
        sleep(1000);
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);

        //Copy code
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

// Raise the lift
        liftMotor1.setPower(0.5);
        liftMotor2.setPower(0.5);
        sleep(1000);
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);
    }
    private void placeCone() {
// Lower the lift
        liftMotor1.setPower(-0.5);
        liftMotor2.setPower(-0.5);
        sleep(1000);
        liftMotor1.setPower(0);
        liftMotor2.setPower(0);

        Copy code
// Open the claw
        clawServo.setPosition(0.5);

// Drive backwards to place the cone
        motorFrontLeft.setPower(-0.5);
        motorFrontRight.setPower(0.5);
        motorBackLeft.setPower(-0.5);
        motorBackRight.setPower(0.5);
        sleep(500);

// Close the claw
        clawServo.setPosition(0);

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