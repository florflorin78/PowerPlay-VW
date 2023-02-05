package org.firstinspires.ftc.teamcode.Autonom;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous(name="RoadRunnerRight")
public class RoadRunnerRight extends LinearOpMode {

    Robot robot = new Robot(hardwareMap);

    public  static double START_X1 = 23.5;
    public  static double START_Y1 = -72;
    public  static double START_HEADING = 90.0;

    Trajectory START_TO_HIGH_PRELOAD;        //start cycle

    Trajectory HIGH_PRELOAD_TO_STACK_CONE1; //cone_cycle_1
    Trajectory STACK_CONE1_TO_HIGH_CONE;    //

    Trajectory HIGH_CONE_TO_STACK_CONE2;    //cone_cycle_2
    Trajectory STACK_CONE2_TO_HIGH_CONE;    //

    Trajectory HIGH_CONE_TO_STACK_CONE3;    //cone_cycle_3
    Trajectory STACK_CONE3_TO_HIGH_CONE;    //

    Trajectory HIGH_CONE_TO_STACK_CONE4;    //cone_cycle_4
    Trajectory STACK_CONE4_TO_HIGH_CONE;    //

    Trajectory HIGH_CONE_TO_PARK1;          // park1
    Trajectory HIGH_CONE_TO_PARK2;          // park2
    Trajectory HIGH_CONE_TO_PARK3;          // park3


    @Override
public void runOpMode() throws InterruptedException {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    robot.GhearaInchide();

    Pose2d startPose = new Pose2d(START_X1, START_Y1, START_HEADING);
    ElapsedTime timer = new ElapsedTime();

    drive.setPoseEstimate(startPose);

    START_TO_HIGH_PRELOAD = drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)), Math.toRadians(40))
                    .addDisplacementMarker(pathLength -> pathLength * 0.3, () -> {

                    })
                    .build();




    HIGH_PRELOAD_TO_STACK_CONE1 = drive.trajectoryBuilder(START_TO_HIGH_PRELOAD.end())
                    .strafeRight(2)
                    .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                    .build();
    STACK_CONE1_TO_HIGH_CONE = drive.trajectoryBuilder(HIGH_PRELOAD_TO_STACK_CONE1.end())
                    .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
                    .build();




    HIGH_CONE_TO_STACK_CONE2 = drive.trajectoryBuilder(STACK_CONE1_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
                    .build();
    STACK_CONE2_TO_HIGH_CONE = drive.trajectoryBuilder(HIGH_CONE_TO_STACK_CONE2.end())
                    .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
                    .build();




    HIGH_CONE_TO_STACK_CONE3 = drive.trajectoryBuilder(STACK_CONE2_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
                    .build();
    STACK_CONE3_TO_HIGH_CONE = drive.trajectoryBuilder(HIGH_CONE_TO_STACK_CONE3.end())
                    .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
                    .build();




    HIGH_CONE_TO_STACK_CONE4 = drive.trajectoryBuilder(STACK_CONE3_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
                    .build();

    STACK_CONE4_TO_HIGH_CONE = drive.trajectoryBuilder(HIGH_CONE_TO_STACK_CONE4.end())
                    .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
                    .build();




    HIGH_CONE_TO_PARK1 = drive.trajectoryBuilder(STACK_CONE4_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(11.5, -11.5, Math.toRadians(90)))
                    .build();

    HIGH_CONE_TO_PARK2 = drive.trajectoryBuilder(STACK_CONE4_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(35.5, -11.5, Math.toRadians(90)))
                    .build();


    HIGH_CONE_TO_PARK3 = drive.trajectoryBuilder(STACK_CONE4_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(58.5, -11.5, Math.toRadians(90)))
                    .build();


        waitForStart();

    if (!isStopRequested()) return;

        drive.followTrajectory(START_TO_HIGH_PRELOAD);




}
    public void LIFT_URCAT(double power, int distance)
    {
        if(opModeIsActive()) {
            robot.LiftStanga.setTargetPosition(distance);
            robot.LiftDreapta.setTargetPosition(distance);

            robot.LiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LiftDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            runtime.reset();
            robot.LiftStanga.setPower(power);
            robot.LiftDreapta.setPower(power);

            while(opModeIsActive() && robot.LiftStanga.isBusy() && robot.LiftDreapta.isBusy())
            {}
            robot.LiftStanga.setPower(0.01);
            robot.LiftDreapta.setPower(-0.01);

            robot.LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void LIFT_COBORAT(double power, int distance)
    {
        if(opModeIsActive()) {
            robot.LiftStanga.setTargetPosition(distance);
            robot.LiftDreapta.setTargetPosition(distance);

            robot.LiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LiftDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            runtime.reset();
            robot.LiftStanga.setPower(-power);
            robot.LiftDreapta.setPower(-power);

            while(opModeIsActive() && robot.LiftStanga.isBusy() && robot.LiftDreapta.isBusy())
            {}
            robot.LiftStanga.setPower(0.01);
            robot.LiftDreapta.setPower(-0.01);

            robot.LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

//    public void UP(double power)
//    {
//        robot.LiftStanga.setPower(power);
//        robot.LiftDreapta.setPower(power);
//    }
//    public void DOWN(double power)
//    {
//        robot.LiftStanga.setPower(-power);
//        robot.LiftDreapta.setPower(-power);
//    }

    public void Stop()
    {
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        sleep(1000);
    }
}
