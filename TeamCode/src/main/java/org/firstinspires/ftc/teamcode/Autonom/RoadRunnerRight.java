package org.firstinspires.ftc.teamcode.Autonom;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

public class RoadRunnerRight extends LinearOpMode {

    Robot robot = new Robot(hardwareMap);

    public  static double START_X1 = 23.5;
    public  static double START_Y1 = -72;
    public  static double START_HEADING = 90.0;

    Trajectory START_TO_HIGH_PRELOAD;

    Trajectory HIGH_PRELOAD_TO_STACK_CONE1;
    Trajectory STACK_CONE1_TO_HIGH_CONE;

    Trajectory HIGH_CONE_TO_STACK_CONE2;
    Trajectory STACK_CONE2_TO_HIGH_CONE;

    Trajectory HIGH_CONE_TO_STACK_CONE3;
    Trajectory STACK_CONE3_TO_HIGH_CONE;

    Trajectory HIGH_CONE_TO_STACK_CONE4;
    Trajectory STACK_CONE4_TO_HIGH_CONE;

    Trajectory HIGH_CONE_TO_PARK1;
    Trajectory HIGH_CONE_TO_PARK2;
    Trajectory HIGH_CONE_TO_PARK3;


    @Override
public void runOpMode() throws InterruptedException {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    robot.GhearaInchide();

    Pose2d startPose = new Pose2d(START_X1, START_Y1, START_HEADING);
    ElapsedTime timer = new ElapsedTime();

    drive.setPoseEstimate(startPose);

    START_TO_HIGH_PRELOAD = drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)), Math.toRadians(40))
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

//    if (!isStopRequested())
//
//        drive.followTrajectorySequence(trajSeq);


}

}
