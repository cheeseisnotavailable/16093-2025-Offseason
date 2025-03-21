package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gobildapinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import XCYOS.Task;

/*
 * The home of SimpleMove, currently for Pinpoint Odometry!
 * With lots of RoadRunner stuff as well
 * Code very much not by me
 */
@Config
public class NewMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANS_PID = new PIDCoefficients(10.4, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(2.1, 0, 0); //i = 0

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private List<DcMotorEx> motors;
    private GoBildaPinpointDriver odo;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    private Runnable updateRunnable;

    boolean manualSwitchDrive = false;
    BooleanSupplier opModeActive = ()->true;

    private double yawHeading = 0;
    public void setUpdateRunnable(Runnable updateRunnable) {
        this.updateRunnable = updateRunnable;
    }
    public void setOpModeActive(BooleanSupplier bs) {
        this.opModeActive = bs;
    }
    private BooleanSupplier switchDrivePIDCondition = ()->false;
    public void setSwitchDrivePIDCondition(BooleanSupplier switchDrivePIDCondition) {
        this.switchDrivePIDCondition = switchDrivePIDCondition;
    }
    private boolean switchDrive = false;

    /**
     * Constructor for NewMecanumDrive.
     * Initialize this during opMode initialization with the opMode's hardwareMap.
     * This version uses the Pinpoint Odometry computer, which requires that the robot is completely still during init.
     * @param hardwareMap
     */
    public NewMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new SQPIDHolonomicFollower(TRANS_PID, TRANS_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-45,115);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.recalibrateIMU();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        setLocalizer(new StandardLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

        odo.recalibrateIMU();

        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (simpleMoveIsActivate) {
            simpleMovePeriod();
        } else if (signal != null) {
            setDriveSignal(signal);
        }
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    /**
     * This must be called every cycle when SimpleMove is active during the update loop of opModes.
     * If SimpleMove is not active, updatePoseEstimate() should be called instead.
     */
    public void update() {
        updatePoseEstimate();

        //This is here to ensure SimpleMove always stops when the opMode stops.
        if(!opModeActive.getAsBoolean()){
            simpleMoveIsActivate = false;
            return;
        }

        //The following line was added by Annie and deals with multiple sets of PIDs.
        //It requires two conditions: one automatic (set in code) and one manual (turned on using useAltPID())
        //Only when both are true will the PID switch.
        switchDrive = switchDrivePIDCondition.getAsBoolean()&&manualSwitchDrive;

        //THE CORE OF SIMPLEMOVE
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (simpleMoveIsActivate) {
            simpleMovePeriod();
        } else if (signal != null) {
            setDriveSignal(signal);
        }

    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            updateRunnable.run();
    }

    /**
     * Used for both SimpleMove and regular RoadRunner
     * When SimpleMove is active it returns true if the position of the robot is within the set tolerance
     * When SimpleMove isn't active it does RoadRunner stuff. (TODO: What RoadRunner stuff?)
     */
    public boolean isBusy() {
        if (simpleMoveIsActivate) {
            Pose2d err = getSimpleMovePosition().minus(getPoseEstimate());
            return  Math.abs(err.getX()) > simpleMove_x_Tolerance || Math.abs(err.getY()) > simpleMove_y_Tolerance || Math.abs(AngleUnit.normalizeRadians(err.getHeading())) > simpleMoveRotationTolerance;
        }
        return trajectorySequenceRunner.isBusy();
    }

    public static PIDCoefficients translationXPid = new PIDCoefficients(0.18, 0.000, 0.006);
    public static PIDCoefficients translationYPid = new PIDCoefficients(0.2, 0.000, 0.01);
    public static PIDCoefficients headingPid = new PIDCoefficients(0.43, 0, 0.09);

    private SQPIDController transPID_x;
    private SQPIDController transPID_y;
    private SQPIDController turnPID;


    public static PIDCoefficients altXPid = new PIDCoefficients(0.30, 0.000, 0.01);
    public static PIDCoefficients altYPid = new PIDCoefficients(0.22, 0.000, 0.009);
    public static PIDCoefficients altHeadingPid = new PIDCoefficients(0.2, 0, 0.2);

    private SQPIDController altTransPID_x;
    private SQPIDController altTransPID_y;
    private SQPIDController altTurnPID;


    private double moveHeading = 0;

    private static final double DEFAULT_TRANS_TOL = 1.25;

    private double simpleMove_x_Tolerance = 1.25, simpleMove_y_Tolerance = 1.25, simpleMoveRotationTolerance = Math.toRadians(10);
    private double simpleMovePower = 0.95;
    public boolean simpleMoveIsActivate = false;

    public void setSimpleMoveTolerance(double x, double y, double rotation) {
        simpleMove_x_Tolerance = x;
        simpleMove_y_Tolerance = y;
        simpleMoveRotationTolerance = rotation;
    }

    public void setSimpleMovePower(double power) {
        simpleMovePower = power;
    }

    public void stopTrajectory() {
        trajectorySequenceRunner.followTrajectorySequenceAsync(null);
        simpleMoveIsActivate = false;
    }

    public void initSimpleMove(Pose2d pos) {
        stopTrajectory();
        simpleMoveIsActivate = true;
        transPID_x = new SQPIDController(translationXPid);
        transPID_x.setSetpoint(pos.getX());

        transPID_y = new SQPIDController(translationYPid);
        transPID_y.setSetpoint(pos.getY());

        turnPID = new SQPIDController(headingPid);
        moveHeading = pos.getHeading();
        turnPID.setSetpoint(0);

        altTransPID_x = new SQPIDController(altXPid);
        altTransPID_x.setSetpoint(pos.getX());

        altTransPID_y = new SQPIDController(altYPid);
        altTransPID_y.setSetpoint(pos.getY());

        altTurnPID = new SQPIDController(altHeadingPid);
        moveHeading = pos.getHeading();
        altTurnPID.setSetpoint(0);
    }

    public void moveTo(Pose2d endPose, int correctTime_ms) {
        long startTime = System.currentTimeMillis();
        initSimpleMove(endPose);
        while (isBusy()) {
            updateRunnable.run();
        }
        long endTime = System.currentTimeMillis() + correctTime_ms;
        while (endTime > System.currentTimeMillis()) {
            updateRunnable.run();
        }
        simpleMoveIsActivate = false;
        setMotorPowers(0, 0, 0, 0);
    }

    public void moveTo(Pose2d endPose, int correctTime_ms, Runnable runWhileMoving) {
        long startTime = System.currentTimeMillis();
        initSimpleMove(endPose);
        while (isBusy()) {
            updateRunnable.run();
            runWhileMoving.run();
        }
        long endTime = System.currentTimeMillis() + correctTime_ms;
        while (endTime > System.currentTimeMillis()) {
            updateRunnable.run();
            runWhileMoving.run();
        }
        simpleMoveIsActivate = false;
        setMotorPowers(0, 0, 0, 0);
    }


    public Pose2d getSimpleMovePosition() {
        return new Pose2d(transPID_x.getSetpoint(), transPID_y.getSetpoint(), moveHeading);
    }

    public Task simpleMoveTime(Pose2d pose, int time, double power, double toleranceRate) {
        return new Task() {
            private long endTime;
            private boolean atReqPos;

            @Override
            public void setUp() {
                atReqPos = toleranceRate<0;
                initSimpleMove(pose);
                simpleMovePower = power;
                simpleMove_x_Tolerance = DEFAULT_TRANS_TOL * toleranceRate;
                simpleMove_y_Tolerance = DEFAULT_TRANS_TOL * toleranceRate;
                endTime = System.currentTimeMillis() + time;
                simpleMoveIsActivate = true;
            }

            @Override
            public void run() {
                if (!atReqPos) {
                    atReqPos = isBusy();
                } else if (endTime < System.currentTimeMillis()) {
                    status = Status.ENDED;
                }
            }

            @Override
            public void end() {
                simpleMoveIsActivate = false;
                simpleMove_x_Tolerance = DEFAULT_TRANS_TOL * toleranceRate;
                simpleMove_y_Tolerance = DEFAULT_TRANS_TOL * toleranceRate;
            }
        };
    }

    public Task simpleMoveTime(Pose2d pose, int time) {
        return simpleMoveTime(pose, time, 0.5, 1);
    }

    public static final double DEAD_BAND = 0.0001;

    public void moveWithDrift(Pose2d... poses) {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // 关闭刹车，允许自由漂移

        for (Pose2d targetPose : poses) {
            moveTo(targetPose,0); // 对每个目标点调用带漂移的移动方法
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 恢复刹车行为
    }

    public void moveTo(Pose2d... poses) {
        for (Pose2d targetPose : poses) {
            moveTo(targetPose,0);
        }
    }



    /**
     *
     *
     * @param drivePower
     * @param x_static
     * @param y_static
     */
    public void setGlobalPower(Pose2d drivePower, double x_static, double y_static) {
        Vector2d vec = drivePower.vec().rotated(-getLocalizer().getPoseEstimate().getHeading());
//        Vector2d vec = drivePower.vec().rotated(-getRawExternalHeading());
        if (vec.norm() > DEAD_BAND) {
            vec = new Vector2d(
                    vec.getX() + Math.copySign(x_static, vec.getX()),
                    vec.getY() + Math.copySign(y_static, vec.getY())
            );
        }
        setWeightedDrivePower(new Pose2d(vec, drivePower.getHeading()));
    }


    public void simpleMovePeriod() {
        Pose2d current_pos = getPoseEstimate();
        if(switchDrive){
            this.setGlobalPower(new Pose2d(
                    clamp(altTransPID_x.calculate(current_pos.getX()), simpleMovePower),
                    clamp(altTransPID_y.calculate(current_pos.getY()), simpleMovePower),
                    clamp(altTurnPID.calculate(AngleUnit.normalizeRadians(current_pos.getHeading() - moveHeading)), simpleMovePower)
            ), 0, 0);
        }else{
            this.setGlobalPower(new Pose2d(
                    clamp(transPID_x.calculate(current_pos.getX()), simpleMovePower),
                    clamp(transPID_y.calculate(current_pos.getY()), simpleMovePower),
                    clamp(turnPID.calculate(AngleUnit.normalizeRadians(current_pos.getHeading() - moveHeading)), simpleMovePower)
            ), 0, 0);
        }
    }


    public Task updatePositionTask = new Task() {
        @Override
        public void run() {
            update();
        }
    };

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            //TODO: CHANGE THE SIGN OF X AND Y
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public static boolean ignoreDriveCoefficients = false;
    public void setFieldCentric(double x, double y, double rx, SuperStructure.Sequences sequence) {
        double botHeading = getHeading();
        double driveCoefficientTrans = 1;
        double driveCoefficientRot = 1;

        if(sequence == SuperStructure.Sequences.INTAKE_FAR){
            driveCoefficientTrans = 0.4;
            driveCoefficientRot = 0.4;
        }else if(sequence == SuperStructure.Sequences.INTAKE_NEAR){
            driveCoefficientTrans = 0.3;
            driveCoefficientRot = 0.3;
        }else if (sequence == SuperStructure.Sequences.LOW_BASKET||sequence==SuperStructure.Sequences.HIGH_BASKET){
            driveCoefficientTrans = 0.9;
            driveCoefficientRot = 0.6;
        } else if (sequence == SuperStructure.Sequences.HIGH_CHAMBER_PLACE){
            driveCoefficientRot = 0.5;
            driveCoefficientTrans = 0.5;
        }else{
            driveCoefficientTrans = 1;
            driveCoefficientRot = 1;
        }

        if(ignoreDriveCoefficients) {
            driveCoefficientTrans = 1;
            driveCoefficientRot = 1;
        }

        y = y*-driveCoefficientTrans;
        x = x*driveCoefficientTrans;
        rx =rx*-driveCoefficientRot;


        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    public void setBotCentric(double x, double y, double rx, SuperStructure.Sequences sequence) {
        double botHeading = 0;
        double driveCoefficientTrans = 1;
        double driveCoefficientRot = 1;

        /*
        Same deal here.
         */

        if(ignoreDriveCoefficients) {
            driveCoefficientTrans = 1;
            driveCoefficientRot = 1;
        }

        y = y*-driveCoefficientTrans;
        x = x*driveCoefficientTrans;
        rx = rx*-driveCoefficientRot;

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        lastEncPositions.add(odo.getEncoderX());
        wheelPositions.add(mmToInches(odo.getPosX()));

        lastEncPositions.add(odo.getEncoderY());
        wheelPositions.add(mmToInches(odo.getPosY()));

        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();

        // TODO: 2024/10/30 getRawVelocity
        lastEncVels.add((int) odo.getVelX());
        wheelVelocities.add(mmToInches(odo.getVelX()));

        lastEncVels.add((int)odo.getVelY());
        wheelVelocities.add(mmToInches(odo.getVelY()));

        return wheelVelocities;
    }

    @Override
    /**
     * Sets motor powers with four doubles.
     */
    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
        rightFront.setPower(rf);
    }

    /**
     * Sets motor powers with an array of length 4.
     * @param powers
     */
    public void setMotorPowers(double[] powers) {
        leftFront.setPower(powers[0]);
        leftRear.setPower(powers[1]);
        rightRear.setPower(powers[2]);
        rightFront.setPower(powers[3]);
    }

    @Override
    public double getRawExternalHeading() {
        return odo.getHeading();
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) odo.getHeadingVelocity();
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    private double clamp(double val, double range) {
        return Range.clip(val, -range, range);
    }
    public static double mmToInches(double mm) {
        return mm/25.4;
    }
    public void updateOdo(){
        odo.update();
    }

    public double getHeading() {
        return odo.getHeading() - yawHeading;
    }

    public void resetHeading(){
        yawHeading = odo.getHeading();
    }

    public void recalibrateOdo(){
        odo.recalibrateIMU();
    }


    public Pose2d lastStoredPos;
    public void storeCurrentPos(){
        if(!simpleMoveIsActivate){
            lastStoredPos = odo.getPositionAsPose2d();
        }
    }
    public String getStoredPosAsString(){
        if(lastStoredPos != null){
            return lastStoredPos.toString();
        }
        return "POSE NOT PROPERLY INITIALIZED!!!!!";
    }

    public String getCurrentPoseAsString(){
        return odo.getPositionAsPose2d().toString();
    }
    public Pose2d getCurrentPose(){
        return odo.getPositionAsPose2d();
    }

    public void testSemicircleRadius(double speedDiff){
        setMotorPowers(1-speedDiff,1-speedDiff,1,1);
    }

    public void useAltPID(boolean on){
        manualSwitchDrive = on;
    }


    public String printMotorSpeeds(){
        String ret = "";
        for(DcMotorEx motor:motors){
            ret += (motor.getCurrentPosition())+" ";
        }
        return ret;
    }
}