package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.Action;
import org.firstinspires.ftc.teamcode.actions.ArmAction;
import org.firstinspires.ftc.teamcode.actions.AscentAction;
import org.firstinspires.ftc.teamcode.actions.GrabAction;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.actions.TailAction;
import org.firstinspires.ftc.teamcode.actions.WristAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.CustomWaitAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.FinishConditionActionGroup;
import org.firstinspires.ftc.teamcode.actions.actioncore.WaitAction;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.ConditionalXCYBoolean;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.TimerBoolean;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//@Photon
public abstract class TeleOpMaster extends LinearOpMode {
    NewMecanumDrive drive;
    SuperStructure upper;
    Pose2d current_pos;
    Runnable update;
    private List<LynxModule> allHubs;
    int count = 0;
    double oldTime = 0;

    // Modes for system control
    int driveMode = 0; // 0: POV mode; 1: Field-centric mode
    public static int slideMode = 0;//1: setpower
    boolean intakeAct = false;
    double slideOpenloopConst = 0.3;
    boolean colorSensorTarget = false;
    DoubleSupplier openLoopSlideController;
    long startTime = Integer.MAX_VALUE;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    XCYBoolean resetPos, resetOdo, changeGrab, slideLonger,slideShorter, forceStop, releaseHigh, releaseLow, switchDrive, autoToggleDriveMode, autoGrabSample, timerPast20, timerPast30, getWallSpecimen, absoluteReset
            , highChamberPlace, highChamberAim, wristHeightSwitch, armDownByPower, manualResetEncoders, goToLastStoredPos, storeThisPos, ascentAim, ascentDown, altWristHeightSwitch, resetArm, manualSlidesBack, altReleaseWrist;
    TimerBoolean touchPressed;

    protected void initTeleOp(BooleanSupplier autoGrabCondition, double startingHeading){
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize SuperStructure with periodic functions for logic and drive control
        upper = new SuperStructure(
                this,
                () -> {
                });

        keybinds();
        timerPast20 = new XCYBoolean(()->System.currentTimeMillis() - startTime > 100000);
        timerPast30 = new XCYBoolean(()->System.currentTimeMillis() - startTime > 110000);

        touchPressed = new TimerBoolean(() -> upper.getTouchSensorPressed(), ()->upper.getSequence() == SuperStructure.Sequences.RUN, 200);
        resetArm = new XCYBoolean(()->touchPressed.trueTimeReached());

        drive = new NewMecanumDrive(hardwareMap);
        drive.setOpModeActive(()->opModeIsActive());
        Action.setOpModeActive(()->opModeIsActive());

        update = () -> {
            logic_period();
            drive_period();
            upper.update();
            gamepad_inputs();


            if (forceStop.toTrue()) {
                Action.stopBuilding = true;
            }
            if (forceStop.toFalse()) {
                Action.stopBuilding = false;
            }

            if(autoGrabSample.toTrue()){
                upper.setIntake(SSValues.CONTINUOUS_STOP);
                Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_CLOSED,40));
                Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 10));
                Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 10));
                gamepad1.rumble(200);
                upper.switchSequence(SuperStructure.Sequences.RUN);
            }

            if(startTime == Integer.MAX_VALUE){
                startTime = System.currentTimeMillis();
            }else if(timerPast30.toTrue()){
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }else if(timerPast20.toTrue()){
                gamepad1.rumble(100);
                gamepad2.rumble(100);
                upper.setAscentState(SuperStructure.AscentState.ASCENT_UP);
            }



            if(drive.simpleMoveIsActivate){
                drive.update();
            }else{
                drive.updatePoseEstimate();
            }

        };

        // Initialize and set up mecanum drive, starting position at (0,0,0)
        drive.setUpdateRunnable(update);
        drive.setPoseEstimate(new Pose2d(-37, 54.5, Math.toRadians(startingHeading)));
        drive.updatePoseEstimate();

        // =====Initial setup for upper mechanisms to default positions=====


//        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_CLOSED);
//        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setAscentState(SuperStructure.AscentState.ASCENT_DOWN);
        upper.setTailPos(SSValues.TAIL_DEFAULT);

//        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.1);
//        upper.setArmByP(SSValues.ARM_DOWN, 0.5);
        upper.unlockSlide();

        drive.storeCurrentPos();
        drive.recalibrateOdo();
        autoToggleDriveMode = new XCYBoolean(() -> upper.getSequence() == SuperStructure.Sequences.HIGH_BASKET && !drive.simpleMoveIsActivate);
        autoGrabSample = new ConditionalXCYBoolean(()->autoGrabCondition.getAsBoolean() && (upper.getWristPosition() == SSValues.WRIST_INTAKE || upper.getWristPosition() == SSValues.WRIST_INTAKE_SPECIMEN_DONTUSETHIS), ()->(upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR));

        upper.setIntake(SSValues.CONTINUOUS_STOP);
        Action.clearActions();
    }

    protected void keybinds(){
        resetPos = new XCYBoolean(() -> (gamepad1.left_stick_button && !gamepad1.right_stick_button) || gamepad1.left_trigger>0);
        resetOdo = new XCYBoolean(() -> gamepad1.a);
        switchDrive = new XCYBoolean(() -> gamepad1.right_stick_button && gamepad1.left_stick_button);
        changeGrab = new XCYBoolean(() -> gamepad1.right_trigger>0.1 && (upper.getSequence() == SuperStructure.Sequences.HIGH_BASKET|| upper.getSequence() == SuperStructure.Sequences.LOW_BASKET));
        slideLonger = new XCYBoolean(() -> gamepad2.dpad_up && !(gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right));
        slideShorter = new XCYBoolean(() -> gamepad2.dpad_down);
        forceStop = new XCYBoolean(() -> gamepad1.b || (gamepad2.back && gamepad2.options));
        releaseHigh = new XCYBoolean(() -> gamepad2.y);
        releaseLow = new XCYBoolean(() -> gamepad2.a);
        highChamberPlace = new XCYBoolean(() -> gamepad1.right_trigger > 0.1 && upper.getSequence() != SuperStructure.Sequences.HIGH_BASKET);
        highChamberAim = new XCYBoolean(() -> gamepad2.left_bumper);
        wristHeightSwitch = new XCYBoolean(() -> gamepad2.right_stick_button);
        altWristHeightSwitch = new XCYBoolean(() -> gamepad2.left_trigger > 1);
        armDownByPower = new XCYBoolean(() -> gamepad2.options && !(gamepad2.back));
        manualSlidesBack = new XCYBoolean(()->gamepad2.back && !gamepad2.options);
        manualResetEncoders = new XCYBoolean(() -> gamepad2.back && gamepad2.options);
        goToLastStoredPos = new XCYBoolean(() -> false);//gamepad1.dpad_left && !(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right));
        storeThisPos = new XCYBoolean(() -> false); //gamepad1.dpad_right && !(gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up));
        ascentAim = new XCYBoolean(()->gamepad2.x);
        ascentDown = new XCYBoolean(()->gamepad1.back);
        getWallSpecimen = new XCYBoolean(()->gamepad2.left_trigger>0);
        absoluteReset = new XCYBoolean(()->gamepad1.touchpad);
        altReleaseWrist = new XCYBoolean(()->gamepad2.touchpad);

        openLoopSlideController = ()->gamepad2.right_stick_y;
    }


    protected void gamepad_inputs() {
        if(touchPressed != null){
            if (Action.actions.isEmpty() && resetArm.toTrue() && (upper.getSequence() == SuperStructure.Sequences.RUN || upper.getSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN)) {
                upper.resetArmEncoder();
            }
        }

        /////////////////////////////// OPERATIONS HANDLING ////////////////////////////////////

        // Accepts inputs only if all previous actions have ended
        if (Action.actions.isEmpty()) {
            // Resets robot position to the "Run" position
            if (resetPos.toTrue()) {
                driveMode = 0;
                upper.switchSequence(SuperStructure.Sequences.RUN);
                upper.setIntake(SSValues.CONTINUOUS_STOP);
                Action.add(new TailAction(upper,TailAction.tailState.TAIL_DEFAULT));
//                    upper.stopIntake();
                // Sequence actions based on last sequence
                if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_CLOSED, 60));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT, 30));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 900));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET ) {
//                    Action.add();(new GrabAction(upper, SSValues.GRAB_DEFAULT,200));
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    Action.add(new WaitAction(200));
//                    Action.add();(new ParallelActionGroup(new ArmAction(upper, SSValues.ARM_LOWER_FROM_BASKET, 50),new SlideAction(upper, SSValues.SLIDE_MIN, 500)));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_LOWER_FROM_BASKET, 270));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 500));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 300));
                }else if(upper.getPreviousSequence() == SuperStructure.Sequences.ASCENT || upper.getPreviousSequence() == SuperStructure.Sequences.RUN){
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    Action.add(new TailAction(upper, TailAction.tailState.TAIL_DEFAULT, 100));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT, 50));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 300));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER_PLACE) {
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 70));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT, 50));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN){
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_CLOSED, 60));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 400));
                    Action.add(new TailAction(upper,TailAction.tailState.TAIL_CHAMBER));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 400));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_AIM_TELEOP));
                    upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER_AIM);
                }
            }

            if(absoluteReset.toTrue()){
                upper.switchSequence(SuperStructure.Sequences.RUN);
                Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
                Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
                upper.setTailPos(SSValues.TAIL_DEFAULT);
                upper.setGrabPos(SSValues.GRAB_DEFAULT);
            }

            // High basket release sequences
            if (releaseHigh.toTrue()) {
                upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
                upper.setGrabPos(SSValues.GRAB_CLOSED);
                drive.storeCurrentPos();
//                Action.add();(new TailAction(upper,SSValues.TAIL_CHAMBER));
//                if(!drive.simpleMoveIsActivate){
//                    driveMode = 1;
//                }
                // Sequence actions for specific release sequences
                if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTERMEDIATE));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP,700));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 400));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_TELEOP));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 100));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_TELEOP));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_CLOSED, 60));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_TELEOP));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX));
                }
            }

            if(getWallSpecimen.toTrue()){
                upper.switchSequence(SuperStructure.Sequences.INTAKE_SPECIMEN);
                upper.setGrabPos(SSValues.GRAB_DEFAULT);
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 30));
                Action.add(new ArmAction(upper, ArmAction.armState.ARM_GET_WALL_SPECIMEN,SSValues.ARM_GET_WALL_SPECIMEN));
                Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE_WALL_SPECIMEN,50));
            }

            // Intake sequences and similar conditional checks...
            if (slideLonger.toTrue()) {
                toggleDriveMode(0);
                upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
//                    drive.storeCurrentPos();
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_ABOVE_SAMPLES));
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR) {
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR));
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET) {
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_ABOVE_SAMPLES));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_ABOVE_SAMPLES));
                }else{
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR));
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                }
            }
            if (slideShorter.toTrue()) {
                toggleDriveMode(0);
                upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
                if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_NEAR));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_ABOVE_SAMPLES));
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_NEAR));
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET) {
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_ABOVE_SAMPLES));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_NEAR));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_ABOVE_SAMPLES));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN){
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_NEAR));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_ABOVE_SAMPLES));
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT));
                }else{
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_NEAR));
                    upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                }
            }

            if (releaseLow.toTrue()) {
                upper.switchSequence(SuperStructure.Sequences.LOW_BASKET);
                upper.setGrabPos(SSValues.GRAB_CLOSED);
                if (upper.getPreviousSequence() == SuperStructure.Sequences.RUN) {
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 500));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_LOW_BASKET));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_TELEOP));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET) {
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_LOW_BASKET));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_TELEOP));
                }
                else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR|| upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER_PLACE) {
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_LOW_BASKET));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_TELEOP));
                } else if (upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_LOW_BASKET));
                }else if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN){
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_CLOSED,80));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP));
                }
            }

            if (highChamberAim.toTrue() && (upper.getSequence() == SuperStructure.Sequences.RUN || upper.getSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN || upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER_PLACE)) {
                upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER_AIM);
                upper.setGrabPos(SSValues.GRAB_CLOSED_WITHOUT_CAP);
                Action.add(new TailAction(upper,TailAction.tailState.TAIL_CHAMBER));
                Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 400));
                Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER));
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_AIM_TELEOP));
            }

            //To place the specimen on the chamber, driver 2 presses the right bumper continuously until it can be released.
            if (highChamberPlace.toTrue()){
                if(upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER_AIM) {
                    upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER_PLACE);
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_PLACE));
                }else if(upper.getSequence() == SuperStructure.Sequences.HIGH_CHAMBER_PLACE) {
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_SPECIMEN);
                    upper.setGrabPos(SSValues.GRAB_OPEN);
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
                    Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE_WALL_SPECIMEN));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_GET_WALL_SPECIMEN, 200));
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT));
                    upper.setTailPos(SSValues.TAIL_DEFAULT);
                }
            }

            //This part allows driver 2 to manually adjust the slide length by power if the upper.getSequence() is intake.
//                if(customSetSlide.toTrue()){
//
//                }
            if ((Math.abs(openLoopSlideController.getAsDouble()) > 0.3) && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN)) {
                slideMode = 1;
                if (upper.getWristPosition() == SSValues.WRIST_INTAKE || upper.getWristPosition() == SSValues.WRIST_INTAKE_SPECIMEN_DONTUSETHIS) {
                    slideOpenloopConst = 0.22   ;
                } else {
                    slideOpenloopConst = 0.7;
                }
                if (openLoopSlideController.getAsDouble() > 0.3 && upper.getSlidesPosition() > SSValues.SLIDE_OPENLOOP_LIMIT) {
                    upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, -openLoopSlideController.getAsDouble() * slideOpenloopConst);
                } else if (openLoopSlideController.getAsDouble() < -0.3 && upper.getSlidesPosition() < SSValues.SLIDE_INTAKE_FAR - SSValues.SLIDE_OPENLOOP_LIMIT) {
                    upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, -openLoopSlideController.getAsDouble() * slideOpenloopConst);
                } else {
                    upper.setSlidesByPower(SSValues.SLIDE_INTAKE_NEAR, 0);
                }
            } else if (upper.getSlideMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                upper.setSlidesByPower(upper.getSlideTargetPosition(), 0);
            }

            //Ascending
            if (slideLonger.toTrue() && upper.getSequence() == SuperStructure.Sequences.LOW_BASKET) {
                upper.switchSequence(SuperStructure.Sequences.ASCENT);
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_ASCENT_UP));
            }
            if (slideShorter.toTrue() && upper.getSequence() == SuperStructure.Sequences.ASCENT) {
                upper.setSlidesByP(SSValues.SLIDE_ASCENT_DOWN, 1);
            }

            //This part allows driver 2 to manually move the arm down.
            if (armDownByPower.toTrue()) {
                upper.switchSequence(SuperStructure.Sequences.RUN);
                upper.setArmByPower(SSValues.ARM_DOWN, -1);
            }
            if (armDownByPower.toFalse()) {
                upper.setArmByPower(SSValues.ARM_DOWN, 0);
            }
//                if(armDownByPower.toFalse()){
//                    upper.resetArmEncoder();
//                }
            if (manualSlidesBack.get()) {
                upper.setSlidesByPower(SSValues.SLIDE_MIN, -1);
            }

            if (manualResetEncoders.toTrue()) {
                drive.simpleMoveIsActivate = false;
                driveMode = 0;
                upper.setWristPos(SSValues.WRIST_DEFAULT);
//                upper.setArmByPower(-SSValues.ARM_HANG1, -1);
            }
            if (manualResetEncoders.toFalse()) {
                upper.resetArmEncoder();
                upper.resetSlideEncoder();
//                drive.resetOdo();
                upper.setArmByPower(SSValues.ARM_DOWN, 0);
                upper.setSlidesByPower(SSValues.SLIDE_MIN, 0);
                gamepad2.rumble(200);
                gamepad1.rumble(200);
                upper.switchSequence(SuperStructure.Sequences.RUN);
            }

            //Reset heading
            if (resetOdo.toTrue()) {
                drive.resetHeading();
            }

            if (gamepad1.right_bumper && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.HIGH_BASKET)) {
                upper.setIntake(SSValues.CONTINUOUS_SPIN);
            } else if (gamepad1.left_bumper && (upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.HIGH_BASKET)) {
                upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
            } else {
                upper.setIntake(SSValues.CONTINUOUS_STOP);
            }

            //Sample released when the arm is in the right place.
            if (changeGrab.toTrue()) {
                if(upper.getSequence() != SuperStructure.Sequences.INTAKE_FAR && upper.getSequence() != SuperStructure.Sequences.INTAKE_NEAR){
                    if (upper.getGrabPos() != SSValues.GRAB_OPEN) {
                        upper.setGrabPos(SSValues.GRAB_OPEN);
                    } else {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                    }
                }else{
                    if (upper.getGrabPos() != SSValues.GRAB_CLOSED) {
                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                        upper.setWristPos(SSValues.WRIST_DEFAULT);
                    } else {
                        upper.setGrabPos(SSValues.GRAB_DEFAULT);
                        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                    }
                }
            }


            if(ascentAim.toTrue()){
                upper.setAscentState(SuperStructure.AscentState.ASCENT_UP);
            }

            if(ascentDown.toTrue()){
                Action.add(new AscentAction(upper, SuperStructure.AscentState.ASCENT_DOWN_A_LITTLE));
                Action.add(new ArmAction(upper, ArmAction.armState.ARM_ASCENT_AIM, 500));
                Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 20));
                Action.add(new AscentAction(upper, SuperStructure.AscentState.ASCENT_DOWN_SOME_MORE));
                Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 20));
                Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 200));
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 750));
                Action.add(new ArmAction(upper, ArmAction.armState.ARM_ASCENT_SWING,50));
                Action.add(new ArmAction(upper, ArmAction.armState.ARM_ASCENT_END, 20));
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 20));
                Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HOLD_ASCENT, 20, 0.1));
                Action.buildSequence(update);
                while(opModeIsActive()){
                    upper.setSlidePower(-1);
                }
            }


            if ((upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR) && wristHeightSwitch.toTrue()) {
                upper.setWristPos(SSValues.GRAB_DEFAULT);
                if (upper.getWristPosition() != SSValues.WRIST_INTAKE) {
                    upper.setWristPos(SSValues.WRIST_INTAKE);
                } else {
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                }
            }
            if ((upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR) && altWristHeightSwitch.toTrue()) {
                upper.setWristPos(SSValues.GRAB_DEFAULT);
                if (upper.getWristPosition() != SSValues.WRIST_INTAKE_SPECIMEN_DONTUSETHIS) {
                    upper.setWristPos(SSValues.WRIST_INTAKE_SPECIMEN_DONTUSETHIS);
                } else {
                    upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
                }
            }

            if(upper.getSequence() == SuperStructure.Sequences.HIGH_BASKET && altReleaseWrist.toTrue()){
                if(upper.getWristPosition() != SSValues.WRIST_INTAKE_WALL_SPECIMEN){
                    upper.setWristPos(SSValues.WRIST_INTAKE_WALL_SPECIMEN);
                }else{
                    upper.setWristPos(SSValues.WRIST_RELEASE_TELEOP);
                }
            }



            if (goToLastStoredPos.toTrue()) {
                driveMode = 2;
                drive.setSimpleMovePower(1);
                drive.setSimpleMoveTolerance(3, 3, Math.toRadians(3));
                drive.moveTo(drive.lastStoredPos, 100, () -> {
                    Action.buildSequence(update);
                    update.run();
                });
                driveMode = 0;
            }


            if(storeThisPos.toTrue()){
                drive.storeCurrentPos();
            }

            if(switchDrive.toTrue()){
                if(driveMode != 0){
                    toggleDriveMode(0);
                }else{
                    toggleDriveMode(1);
                }
            }
        }
    }

    //AAAAAAAAAAAAA
    protected void scorePresetSamples(){

        upper.setGrabPos(SSValues.GRAB_CLOSED);
        delay(10);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

        driveMode = 2;

        Pose2d blueBasket = new Pose2d(52.7, 51.3, Math.toRadians(-120));
        drive.setSimpleMovePower(0.7);

        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 400));
        Action.add(new WaitAction(1200));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 900));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, (int)(SSValues.ARM_UP*0.3)));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, SSValues.SLIDE_MAX));
        Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.4),0));
        drive.setSimpleMoveTolerance(1.5, 1.5, Math.toRadians(7));
        drive.moveTo(blueBasket, 50, ()->Action.buildSequence(update));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_EXTRA,340));
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN));
        Action.buildSequence(update);
        delay(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);

        resetAfterBlueBasketAndMoveToIntake(0, 0.5, 10);
        expGetYellowSamples();

        putBlueBaksetFromGround(2,0, 0);
        resetAfterBlueBasketAndMoveToIntake(7.2, 3.4, 10);

        expGetYellowSamples();
        putBlueBaksetFromGround(0,1, 0);

        moveAndIntakeLastBasketSampleBlue();
        putBlueBaksetFromGround(0,1, 0);

        driveMode = 0;
    }

    private Pose2d yellowPose;
    private void resetAfterBlueBasketAndMoveToIntake(double xOffset, double yOffset, double headingOffset){
        yellowPose = new Pose2d(46+xOffset, 47.6+yOffset, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);

        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, (int)(SSValues.SLIDE_MAX*0.75)));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.moveTo(yellowPose, 110, ()->Action.buildSequence(update));
    }

    private Pose2d lastBlueBasketSample = new Pose2d(52.8,46.0,Math.toRadians(-49));
    private void moveAndIntakeLastBasketSampleBlue(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(5));
        drive.setSimpleMovePower(0.3);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 950));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
//        moveToGetLastYellowSample();
        drive.moveTo(lastBlueBasketSample, 150, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_AUTO_INTAKE_LAST_BLUE,20,0.5),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);}));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 500));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);

    }
    private void putBlueBaksetFromGround(double xOffset, double yOffset, double simpleMovePowerChange){
        Pose2d blueBasket = new Pose2d(50+xOffset, 57.4+yOffset, Math.toRadians(-120));
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.3 + simpleMovePowerChange);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 800));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 100));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_AUTO,340));
        drive.moveTo(blueBasket, 270,()->Action.buildSequence(update));
//        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE, 20));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN));
        Action.buildSequence(update);
        sleep(150);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    private void expGetYellowSamples(){
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(3));
        drive.setSimpleMovePower(0.65);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE,0));
        Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_AUTO_INTAKE_YELLOW,20,0.35),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);}));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setGrabPos(SSValues.GRAB_CLOSED);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
        Action.buildSequence(update);
    }





    private void drive_period() {
        if (upper != null) {
            if (driveMode == 0) {
                drive.setFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, upper.getSequence());
            }else if (driveMode == 1){
                drive.setBotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, upper.getSequence());
            }
            drive.updateOdo();
        }
    }

    private void toggleDriveMode(int mode) {
        if (!drive.simpleMoveIsActivate) {
            driveMode = mode;
        }
    }


    // Logic updates with telemetry
    protected void logic_period() {
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        XCYBoolean.bulkRead();
//        count ++;
//        telemetry.addData("Loops since start: ", count);
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
//
//        telemetry.addData("Arm Position: ", upper.getArmPosition());
//        telemetry.addData("Slide Position: ", upper.getSlidesPosition());
//        telemetry.addLine("");
//        telemetry.addData("Arm Power", upper.getArmPower());
//        telemetry.addData("Slide Power:", upper.getSlidePower());
//        telemetry.addLine("");
//
//        telemetry.addData("Arm Target Position", upper.getArmTargetPosition());
//        telemetry.addData("Slide Target Position", upper.getSlideTargetPosition());
//        telemetry.addLine("");
//        telemetry.addData("Current Sequence", upper.getSequence());
//        telemetry.addData("Previous Sequence", upper.getPreviousSequence());
//        telemetry.addLine("");
//        telemetry.addData("Drive Mode", driveMode);
//        telemetry.addData("Action Stop?", Action.stopBuilding);
//        telemetry.addData("Touch Sensor Pressed?", upper.mTouchSensor.isPressed());
//        telemetry.addData("Touch Sensor TrueTimeReached", touchPressed.trueTimeReached());
//        telemetry.addData("Last Stored Pose:", drive.getStoredPosAsString());
//        if (upper.getSequence() == SuperStructure.Sequences.RUN)
//            telemetry.addData("Current Pos", drive.getCurrentPoseAsString());
//        telemetry.addData("DriveMode: ", driveMode);

//        telemetry.addData("Slide Lock Position", upper.getSlideLockPosition());
//        telemetry.addData("Color Sensor values",upper.getColorRGBAValues(15));
//        telemetry.addData("AutoGrab: ", autoGrabSample.get());
//        telemetry.addData("AutoGrab toTrue: ", autoGrabSample.toTrue());
//        if(upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
//            telemetry.addData("Detected Sample Color", upper.alphaAdjustedSampleColor());
//            NormalizedRGBA rgba = upper.getColorRGBAValues(5);
//            telemetry.addLine(String.format("rgba: %f %f %f %f",rgba.red, rgba.green, rgba.blue, rgba.alpha));
//            telemetry.addData("Is there a sample?", upper.colorSensorCovered());
//        }
//        telemetry.addLine(Action.showCurrentAction());
//        telemetry.update();

//        telemetry_M.addData("Slide Power:", upper.getSlidePower());
//        telemetry_M.addData("Arm Power", upper.getArmPower());
//        telemetry_M.update();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }


    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            update.run();
        }
    }
}