package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotHardwareEssentials {

    //constants
    public static final Double TRIGGER_THRESHOLD = 0.5; //gamepad trigger

    //drive base motors
    public DcMotor LeftFront;
    public DcMotor LeftBack;
    public DcMotor RightFront;
    public DcMotor RightBack;

    //odometers
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    private HardwareMap hardwareMap;

    public RobotHardwareEssentials(HardwareMap aHardwareMap) {
        hardwareMap = aHardwareMap;

        //configure the drive motors
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightBack = hardwareMap.dcMotor.get("RightBack");
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //shadow the motors with the odo encoders
        encoderLeft = LeftBack;
        encoderRight = RightBack;
        encoderAux = RightFront;

        stop();
        resetDriveEncoders();

    }


    public void resetDriveEncoders() {

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void stop() {

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

    }
    public class XyhVector {
        public double x;
        public double y;
        public double h;

        public XyhVector(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
        public XyhVector(XyhVector v) {
            this.x = v.x;
            this.y = v.y;
            this.h = v.h;
        }
    }

        //constants that define the geometry of the robot :
        final static double L = 25; //distance between encoder 1 and 2 in cm
        final static double B = 28.4; //distance between the midpoint 1 and 2 and encoder 3
        final static double R = 4.8; //wheel radius in cm
        final static double N = 3200; //encoder ticks per revolution,
        final static double cm_per_tick = 2.0 * Math.PI * R / N;

        //keep track of the odometry encoders between updates:
        public int currentRightPosition = 0;
        public int currentLeftPosition = 0;
        public int currentAuxPosition = 0;

        private int oldRightPosition = 0;
        private int oldLeftPosition = 0;
        private int oldAuxPosition = 0;

        public XyhVector START_POS = new XyhVector(213, 102, Math.toRadians(-174));
        public XyhVector pos = new XyhVector(START_POS);

        public void odometry() {
            oldRightPosition = currentRightPosition;
            oldLeftPosition = currentLeftPosition;
            oldAuxPosition = currentAuxPosition;

            currentRightPosition = -encoderRight.getCurrentPosition();
            currentLeftPosition = -encoderLeft.getCurrentPosition();
            currentAuxPosition = encoderAux.getCurrentPosition();

            int dn1 = currentLeftPosition - oldLeftPosition;
            int dn2 = currentRightPosition - oldRightPosition;
            int dn3 = currentAuxPosition - oldAuxPosition;

            //the robot has moved and turned a tiny bit between two measurements
            double dtheta = cm_per_tick * (dn2 - dn1) / L;
            double dx = cm_per_tick * (dn1 + dn2) / 2.0;
            double dy = cm_per_tick * (dn3 - (dn2 - dn1)) * B / L;

            //small movement of the robot gets added to the field coordonate system:
            double theta = pos.h + (dtheta / 2.0);
            pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
            pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
            pos.h += dtheta;


        }
    }
