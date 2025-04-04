package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.CancellableFinishConditionActionGroup;
import org.firstinspires.ftc.teamcode.actions.actioncore.DriveAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.FinishConditionActionGroup;
import org.firstinspires.ftc.teamcode.actions.IntakeAction;
import org.firstinspires.ftc.teamcode.actions.TailAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.CustomWaitAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.SequentialActionGroup;
import org.firstinspires.ftc.teamcode.actions.actioncore.WaitAction;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.TimerBoolean;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.actions.actioncore.Action;
import org.firstinspires.ftc.teamcode.actions.ArmAction;
import org.firstinspires.ftc.teamcode.actions.GrabAction;
import org.firstinspires.ftc.teamcode.actions.actioncore.ParallelActionGroup;
import org.firstinspires.ftc.teamcode.actions.SlideAction;
import org.firstinspires.ftc.teamcode.actions.WristAction;
import org.firstinspires.ftc.teamcode.util.CentralLogController;

import java.util.List;

@Config
public abstract class AutoMaster extends LinearOpMode {

    private NewMecanumDrive drive;
    protected SuperStructure upper;
    protected Runnable update;
    private List<LynxModule> allHubs;

    Pose2d startPos;

    double oldTime = 0;
    int loopCount = 0;
    static long startTime;
    boolean resetOnce = false;


    protected void initAuto(Pose2d start) throws InterruptedException{
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        startPos = start;
        startTime = System.currentTimeMillis();
        Action.clearActions();
        Action.setOpModeActive(()->opModeIsActive());

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new NewMecanumDrive(hardwareMap);
        drive.recalibrateOdo();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(startPos);
        drive.setOpModeActive(()->opModeIsActive());
        drive.updatePoseEstimate();
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(10));

        telemetry.addLine("init: superstructure");
        telemetry.update();
        upper = new SuperStructure(
                this,
                () -> {
                });

        TimerBoolean touchPressed = new TimerBoolean(() -> upper.getTouchSensorPressed(), ()->upper.getSequence() == SuperStructure.Sequences.RUN || upper.getSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN, 50);
//        XCYBoolean resetArm = new XCYBoolean(()->touchPressed.trueTimeReached());

        update = ()->{
            drive.update();
            upper.update();
            XCYBoolean.bulkRead();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
            loopCount ++;
            telemetry.addData("Loops since start: ", loopCount);
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

//            if(drive.simpleMoveIsActivate){
//                SimpleLogUtil.add("SimpleMove at "+drive.getPoseEstimate().toString()+" SimpleMove target "+drive.getSimpleMovePosition().toString());
//            }
//            if (!Action.actions.isEmpty()) {
//                SimpleLogUtil.add("Running " + Action.showCurrentAction());
//            }


//            telemetry.addData("Switch PID?", upper.slideTooHigh);
//            telemetry.addData("Start Time: ", drive.startTime);
//            telemetry.addData("Time since last MoveTo: ", drive.millisSinceMoveTo);
//            telemetry.addData("Current Pose", drive.getCurrentPoseAsString());
//            telemetry.addData("Target Pose",(drive.simpleMoveIsActivate)? drive.getSimpleMovePosition().toString() : "SimpleMove not activated");
//            telemetry.addData("Arm Position: ", upper.getArmPosition());
//            telemetry.addData("Arm Target: ", upper.getArmTargetPosition());
//            telemetry.addData("Slide Target: ", upper.getSlideTargetPosition());
//            telemetry.addData("Slide Error: ", upper.getSlideError());
//            telemetry.addData("Slide Power: ", upper.getSlidePower());
//            telemetry.addData("Current Sequence: ",upper.getSequence());
//            telemetry.addData("Current Time: ", System.currentTimeMillis()-startTime);
//            telemetry.addData("Touch Sensor Pressed?", upper.getTouchSensorPressed());
//            telemetry.addData("Touch time since true: ", touchPressed.getTimeSinceTrue());
//            telemetry.addData("Touch true time reached: ", touchPressed.trueTimeReached());
//
//
//            if(upper.getSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
//                telemetry.addData("Detected Sample Color", upper.colorOfSample());
//                telemetry.addData("Color Raw Values", upper.getColorRGBAValues(5).toString());
////            telemetry.addData("Is there a sample?", upper.colorSensorCovered());
//            }
//
//            telemetry.addLine(Action.showCurrentAction());
//            telemetry.update();
            if(!resetOnce){
                if (Math.abs(upper.getArmError()) < 30 && upper.getArmTargetPosition() == SSValues.ARM_DOWN-upper.armOffset) {
                    if (touchPressed.trueTimeReached() && (upper.getSequence() == SuperStructure.Sequences.RUN || upper.getSequence() == SuperStructure.Sequences.INTAKE_SPECIMEN)){
                        if(Action.currentAction.returnType().equals("ArmAction")){
                            Action.currentAction.stop();
                        }
                        upper.armOffset = 0;
                        upper.resetArmEncoder();
                        resetOnce = true;
                    }
                }
            }
            //TODO: THIS DOESN'T WORK
//            if(drive.simpleMoveInDistress){
//                prepareForTeleOpWithoutMoving();
//            }
//            if(isStopRequested()){
//                Action.stopBuilding = true;
//            }

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        };

        drive.setUpdateRunnable(update);
        drive.setSwitchDrivePIDCondition(()->true);

        upper.resetSlide();
        upper.setGrabPos(SSValues.AUTO_GRAB_CLOSED);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setTailPos(SSValues.TAIL_DEFAULT);
        upper.setAscentState(SuperStructure.AscentState.ASCENT_DOWN);

//        upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
//        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);

        telemetry.addLine("init: complete");
        telemetry.update();


    }

    protected void setStartTime(){
        startTime = System.currentTimeMillis();
    }

    ///////////////////////////////////BLUE BASKET///////////////////////////////////////////////
    protected void reset(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        // Sequence actions based on last sequence
        if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR) {
            upper.setGrabPos(SSValues.AUTO_GRAB_CLOSED);
            Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
            Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 10));
        } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.ASCENT || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
            upper.setGrabPos(SSValues.GRAB_DEFAULT);
            Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 100));
            Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
            Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT, 50));
            Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 300));
        }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER_PLACE){
            Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER_RESET, 100));
            Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
            Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,200));
            Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        }
        Action.buildSequence(update);
    }

    protected void newResetFromHighChamber(){
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER_RESET, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, (int)(0.8*SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO)));
        Action.buildSequence(update);
    }
    protected void newResetCompletelyFromHighChamber(){
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER_RESET, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,SSValues.ARM_UP));
        Action.buildSequence(update);
    }
    protected Pose2d yellowPose;
    protected void resetAfterBlueBasketAndMoveToIntake(double xOffset, double headingOffset){
        yellowPose = new Pose2d(47.5+xOffset, 47, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.75);
        drive.moveTo(new Pose2d(50, 45, Math.toRadians(-135)), 20);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_ABOVE_SAMPLES, 100));
        Action.buildSequence(update);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 300));
        drive.moveTo(yellowPose, 600, ()->Action.buildSequence(update));
    }

    protected void expResetAfterRedBasketAndMoveToIntake(double xOffset, double yOffset, double headingOffset){
        addLogMarker("reset after red basket and move to intake");

        yellowPose = new Pose2d(-48+xOffset, -47.6+yOffset, Math.toRadians(90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);

        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, (int)(SSValues.SLIDE_MAX*0.75)));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.moveTo(yellowPose, 110, ()->Action.buildSequence(update));
    }

    protected void expResetAfterBlueBasketAndMoveToIntake(double xOffset, double yOffset, double headingOffset){
        yellowPose = new Pose2d(47+xOffset, 48.7+yOffset, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);

        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, (int)(SSValues.SLIDE_MAX*0.75)));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.moveTo(yellowPose, 100, ()->Action.buildSequence(update));
    }

    protected void turn(double angleInDegrees, int correctMs){
        drive.moveTo(new Pose2d(drive.getCurrentPose().getX(), drive.getCurrentPose().getY(), drive.getCurrentPose().getHeading()+Math.toRadians(angleInDegrees)), correctMs);
    }

    protected void turnTo(double angleInDegrees, int correctMs){
        drive.moveTo(new Pose2d(drive.getCurrentPose().getX(), drive.getCurrentPose().getY(), Math.toRadians(angleInDegrees)), correctMs);
    }

    protected void expResetChamberAndMoveToIntake(double xOffset, double yOffset, double headingOffset, boolean wait){
        addLogMarker("expResetChamberAndMoveToIntake");
        bluePose = new Pose2d(-50+xOffset, 49.7+yOffset, Math.toRadians(-90+headingOffset));
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(0.8,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.7);

        if(wait){
            Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
            Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE_WALL_SPECIMEN));
            Action.add(new ArmAction(upper, ArmAction.armState.ARM_GET_WALL_SPECIMEN,400));
        }
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, (int)(SSValues.SLIDE_MAX*0.75)));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.moveTo(bluePose, 60, ()->Action.buildSequence(update));
    }


    protected void newFirstMoveToBlueChamberPlace(){
        addLogMarker("newFirstMoveToBlueChamberPlace");
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(0.8);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER_PLACE);
        Action.add(new TailAction(upper, TailAction.tailState.TAIL_CHAMBER));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 900));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_AIM_AUTO,300));
        Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.3),0));
//        Action.add();(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,150));
        drive.moveTo(new Pose2d(-11.3, 35.5, Math.toRadians(90)),10, ()->Action.buildSequence(update));
//        drive.moveTo(new Pose2d(-10, 39.3, Math.toRadians(90)),50,()->Action.buildSequence(update));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_PLACE_AUTO,130));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN,50));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
        Action.add(new TailAction(upper, TailAction.tailState.TAIL_DEFAULT, 130));
        Action.buildSequence(update);
    }

    protected void blueChamberPlaceFromWall(double xOffset, double yOffset){
        addLogMarker("blueChamberPlaceFromWall");
        drive.useAltPID(true);
        drive.setSimpleMoveTolerance(1,3, Math.toRadians(8));
        drive.setSimpleMovePower(0.9);

        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 10000));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 1050));
        Action.buildSequence(update);

        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER_PLACE);

        Action.add(new TailAction(upper,TailAction.tailState.TAIL_CHAMBER));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER, 800));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_AIM_AUTO, 300));

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(-10+xOffset, 42+yOffset, Math.toRadians(90)),0,()->{Action.buildSequence(update);}); //42
        drive.setSimpleMovePower(0.7);
        drive.moveTo(new Pose2d(-10+xOffset, 34+yOffset, Math.toRadians(90)),10);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_PLACE_AUTO,130));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        drive.useAltPID(false);
    }

    protected void redChamberPlaceFromWall(double xOffset, double yOffset){
//        drive.turnOnSwitchDrive(true);
        drive.setSimpleMoveTolerance(2,3, Math.toRadians(8));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 980));
        Action.buildSequence(update);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER_PLACE);
        Action.add(new TailAction(upper,TailAction.tailState.TAIL_CHAMBER));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_AIM_AUTO));
        drive.moveTo(new Pose2d(8+xOffset, -43.5+yOffset, Math.toRadians(-90)),20,()->{Action.buildSequence(update);});
        drive.moveTo(new Pose2d(8+xOffset, -33.5+yOffset, Math.toRadians(-90)),10, ()->drive.setSimpleMovePower(0.5));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_PLACE_AUTO,150));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        drive.turnOnSwitchDrive(false);
    }




    protected void intakeSpecimenFromGround(double xOffset, double yOffset){
        drive.setSimpleMovePower(0.95);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,600));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_AUTO_INTAKE_YELLOW,10, 0.8));
        drive.moveTo(new Pose2d(-20+xOffset, 45+yOffset, Math.toRadians(135)), 10, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
        Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR,20,0.15),
                ()->upper.colorSensorCovered(),
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);},
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);}));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_CLOSED));
//        Action.add();(new SlideAction(upper, SSValues.SLIDE_INTAKE_FAR,10,0.3));
//        Action.buildSequence(update);
//        delay(40);
//        Action.add();(new GrabAction(upper, SSValues.GRAB_CLOSED));
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void intakeSpecimenFromBlueWall(double xOffset, double yOffset){
        drive.useAltPID(true);
        addLogMarker("intakeSpecimenFromBlueWall");
//        drive.turnOnSwitchDrive(false);
        drive.setSimpleMovePower(0.9);
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(7));
        upper.switchSequence(SuperStructure.Sequences.INTAKE_SPECIMEN);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_GET_WALL_SPECIMEN,800));
        drive.moveTo(new Pose2d(-37.5+xOffset,51.5+yOffset,Math.toRadians(90)), 20, ()->Action.buildSequence(update));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE_WALL_SPECIMEN));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT));
        Action.add(new SlideAction(upper,SlideAction.slideState.SLIDE_INTAKE_WALL_SPECIMEN,10));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_CLOSED);
//        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_CLOSED,30));
//        Action.buildSequence(update);

        drive.useAltPID(false);
    }

    protected void intakeSpecimenFromRedWall(double xOffset, double yOffset){
//        drive.turnOnSwitchDrive(false);
        drive.setSimpleMovePower(1);
        drive.setSimpleMoveTolerance(4,1,Math.toRadians(7));
        upper.switchSequence(SuperStructure.Sequences.INTAKE_SPECIMEN);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE_WALL_SPECIMEN));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_GET_WALL_SPECIMEN,400));
        drive.moveTo(new Pose2d(34.5+xOffset,-47.8+yOffset,Math.toRadians(-90)), 10, ()->Action.buildSequence(update));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT,180));
        Action.add(new SlideAction(upper,SlideAction.slideState.SLIDE_INTAKE_WALL_SPECIMEN,10, 0.6));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_CLOSED,80));
        Action.buildSequence(update);
//        Action.add();(new SlideAction(upper, SSValues.SLIDE_MIN, 50));
//        Action.buildSequence(update);
    }

    protected void newFirstMoveToRedChamberPlace(){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER_PLACE);
        Action.add(new TailAction(upper, TailAction.tailState.TAIL_CHAMBER));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 900));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_HIGH_CHAMBER));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_AIM_AUTO,700));
        Action.add(new WaitAction(150));
        Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.25),0));
//        Action.add();(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE_AUTO,150));
        drive.moveTo(new Pose2d(9.2, -33, Math.toRadians(-90)),70, ()->Action.buildSequence(update));
//        drive.moveTo(new Pose2d(-10, 39.3, Math.toRadians(90)),50,()->Action.buildSequence(update));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_PLACE_AUTO,130));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN,80));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.add(new TailAction(upper, TailAction.tailState.TAIL_DEFAULT, 130));
        Action.buildSequence(update);
    }


    protected void parkFromRedChamber(){
//        drive.turnOnSwitchDrive(false);
        drive.setSimpleMovePower(1);
        drive.setSimpleMoveTolerance(4,1,Math.toRadians(7));
        upper.switchSequence(SuperStructure.Sequences.INTAKE_SPECIMEN);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setTailPos(SSValues.TAIL_DEFAULT);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,400));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(drive.getCurrentPose().getX(),-38,Math.toRadians(-90)), 10, ()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(38.5,-50.9,Math.toRadians(90)), 10);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    protected void parkFromBlueChamber(){
//        drive.turnOnSwitchDrive(false);
        drive.setSimpleMovePower(1);
        drive.setSimpleMoveTolerance(4,1,Math.toRadians(7));
        upper.switchSequence(SuperStructure.Sequences.INTAKE_SPECIMEN);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setTailPos(SSValues.TAIL_DEFAULT);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 1400));
        Action.buildSequence(update);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,400));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(drive.getCurrentPose().getX(),38,Math.toRadians(90)), 10);
        drive.moveTo(new Pose2d(-38.5,50.9,Math.toRadians(-90)), 10, ()->Action.buildSequence(update));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    protected void newRedChamberPlace(double xOffset){
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(7));
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER_PLACE);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.add(new ParallelActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER,600),new ArmAction(upper, ArmAction.armState.ARM_UP, 700)));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_AIM_AUTO,10));
        drive.moveTo(new Pose2d(-10+xOffset, 37, Math.toRadians(90)),20,()->Action.buildSequence(update));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_HIGH_CHAMBER_PLACE,100));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
    }

    protected void newParkFromRedChamber(){
        drive.setSimpleMovePower(1);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,400));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN,100));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE,20));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(-20, 48, Math.toRadians(135)), 0, ()->Action.buildSequence(update));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR,100));
        Action.buildSequence(update);
    }


    protected Pose2d blueBasket = new Pose2d(52, 54.5, Math.toRadians(-135));
    protected Pose2d redBasket = new Pose2d(-53.3, -56.3, Math.toRadians(45));

    protected void expFirstPutBlueBasket(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.65);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 0));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 90));
        Action.buildSequence(update);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 700));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 90));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 90));
        drive.moveTo(new Pose2d(54, 54, Math.toRadians(-130)), 200,()-> {
            Action.buildSequence(update);
            Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 90));
            Action.buildSequence(update);}
        );
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_TELEOP,200));
        Action.buildSequence(update);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_TELEOP,40));
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE, 10));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN));
        Action.buildSequence(update);
        upper.setGrabPos(SSValues.GRAB_OPEN);
        delay(150);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void expFirstPutRedBasket(){
        addLogMarker("expFirstPutRedBasket");
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.65);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 0));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 90));
        Action.buildSequence(update);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 700));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 90));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 90));
//        Action.buildSequence(update);
        drive.moveTo(new Pose2d(-53.5, -57.5, Math.toRadians(50)), 200,()-> {
            Action.buildSequence(update);
            Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 90));
            Action.buildSequence(update);}
        );
//        sleep(400);
        upper.setWristPos(SSValues.WRIST_RELEASE_AUTO);
        delay(50);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_AUTO,200));
        Action.buildSequence(update);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_AUTO,100));
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE, 10));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN, 100));
        Action.buildSequence(update);
        delay(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void throwBehind(){
        addLogMarker("throwBehind");
        upper.switchSequence(SuperStructure.Sequences.LOW_BASKET);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 500));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 100));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_AUTO));
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN));
        Action.buildSequence(update);
        sleep(400);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putBlueBasketFromGround(double xOffset, double yOffset, double simpleMovePowerChange){
        blueBasket = new Pose2d(50.7+xOffset, 54.3+yOffset, Math.toRadians(-125));
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.3 + simpleMovePowerChange);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 800));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 100));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_AUTO,340));
        drive.moveTo(blueBasket, 250,()->Action.buildSequence(update));
//        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE, 20));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN));
        Action.buildSequence(update);
        sleep(150);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putRedBasketFromGround(double xOffset, double yOffset, double simpleMovePowerChange){
        addLogMarker("putRedBasketFromGround");

        redBasket = new Pose2d(-50+xOffset, -57.4+yOffset, Math.toRadians(60));
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        drive.setSimpleMovePower(0.3 + simpleMovePowerChange);
        upper.setWristPos(SSValues.WRIST_INTAKE);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 800));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 100));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_AUTO,340));
        drive.moveTo(redBasket, 270,()->Action.buildSequence(update));
//        upper.setIntake(SSValues.CONTINUOUS_SPIN_OPPOSITE);
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE, 20));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN));
        Action.buildSequence(update);
        sleep(150);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putBlueBasketFromSubmersible(double xOffset, double yOffset, double degreeOffset){
        blueBasket = new Pose2d(50.7+xOffset, 54.3+yOffset, Math.toRadians(-110));
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 400));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 900));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.setSimpleMovePower(0.7);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, (int)(SSValues.ARM_UP*0.3)));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, SSValues.SLIDE_MAX));
        Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.4),0));
        drive.setSimpleMoveTolerance(1.5, 1.5, Math.toRadians(7));
        drive.moveTo(new Pose2d(blueBasket.getX()-2, blueBasket.getY()-2, Math.toRadians(-110+degreeOffset)), 0,()->{Action.buildSequence(update);drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);});
        drive.moveTo(blueBasket, 50);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_EXTRA,340));
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN));
        Action.buildSequence(update);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delay(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    protected void putRedBasketFromSubmersible(double xOffset, double yOffset, double degreeOffset){
        addLogMarker("putToSubmersibleRed");
        drive.setSimpleMoveTolerance(3, 3, Math.toRadians(5));
        redBasket = new Pose2d(-52.5+xOffset, -58+yOffset, Math.toRadians(60));
        upper.switchSequence(SuperStructure.Sequences.HIGH_BASKET);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.setSimpleMovePower(0.7);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 300));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, 900));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP, (int)(SSValues.ARM_UP*0.2)));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MAX, 1200));
        Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.3),0));
        drive.setSimpleMoveTolerance(1.5, 1.5, Math.toRadians(7));
        drive.moveTo(new Pose2d(redBasket.getX(), redBasket.getY(), Math.toRadians(80+degreeOffset)), 80,()->{Action.buildSequence(update);drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);});
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_RELEASE_EXTRA,340));
        Action.add(new IntakeAction(upper, IntakeAction.intakeState.CONTINUOUS_SPIN_OPPOSITE));
        Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_OPEN));
        Action.buildSequence(update);
        delay(100);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }




    // Method to move the robot with proportional deceleration and motor power adjustment
    private void moveToWithDeceleration(Pose2d target, double speed) {
        // Get the current position of the robot
        Pose2d currentPose = drive.getPoseEstimate();

        // Calculate the distance to the target
        double targetY = target.getY();
        double currentY = currentPose.getY();

        double distanceToTarget = Math.abs(targetY - currentY);

        // Define the maximum speed
        double maxSpeed = speed;  // You can adjust the max speed here

        // Calculate the speed using proportional deceleration based on remaining distance
        double adjustedSpeed = calculateSpeed(distanceToTarget, maxSpeed);

        // Convert the adjusted speed into motor power
        double motorPower = speedToMotorPower(adjustedSpeed);

        // Move to the target with adjusted motor power
        drive.setMotorPowers(motorPower, motorPower,motorPower,motorPower);
        drive.moveTo(new Pose2d(target.getX(),
                target.getY(),
                target.getHeading()),0);
    }

    // Function to calculate the robot's speed based on distance to the target
    private double calculateSpeed(double distanceToTarget, double maxSpeed) {
        // Proportional deceleration: speed decreases as distance decreases
        double speed = maxSpeed * (distanceToTarget / 5);  // 5 is the maximum distance to decelerate (you can adjust this value)

        // Ensure the speed doesn't go below a minimum threshold
        if (speed < 0.1) {  // You can adjust the minimum speed
            speed = 0.1;
        }

        return speed;
    }

    // Convert speed to motor power (scale to -1 to 1 range)
    private double speedToMotorPower(double speed) {
        // Assuming the max speed is normalized to 1.0, and we want the motor power to be proportional to this speed
        // Ensure the motor power is within -1.0 and 1.0
        double motorPower = speed;  // Since speed is normalized between 0 and maxSpeed, motor power can directly match it

        // Optional: Apply a deadzone to avoid jittering at low speeds (you can tune this value)
        if (Math.abs(motorPower) < 0.1) {
            motorPower = 0;  // Deadzone to prevent small motor movements
        }

        return motorPower;
    }

    protected void VexpPushTwoRedSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(5,5, Math.toRadians(20));
        drive.setSimpleMovePower(1);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 900));
        Action.buildSequence(update);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,200));
        drive.moveTo(new Pose2d(32, -40, Math.toRadians(-90)), 0,()->Action.buildSequence(update));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,200));
        drive.moveWithDrift(new Pose2d(34, -13, Math.toRadians(-90)),
                new Pose2d(43, -13, Math.toRadians(-90)),
                new Pose2d(43, -48, Math.toRadians(-90)),
                new Pose2d(43, -12, Math.toRadians(-90)),
                new Pose2d(54, -12, Math.toRadians(-90)),
                new Pose2d(55, -49, Math.toRadians(-90)),
                new Pose2d(54, -12, Math.toRadians(-90)),
                new Pose2d(59, -12, Math.toRadians(-90)),
                new Pose2d(59, -49, Math.toRadians(-90)));
    }

    protected void VexpPushTwoBlueSamples(){
        addLogMarker("VexpPushTwoBlueSamples");
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(2,2, Math.toRadians(10));
        drive.setSimpleMovePower(1);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 900));
        Action.buildSequence(update);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN,1000));
        Action.buildSequence(update);
        drive.moveTo(
                new Pose2d(-36, 40, Math.toRadians(90)),
                new Pose2d(-36, 15, Math.toRadians(90)),
                new Pose2d(-46, 17, Math.toRadians(90)));
        drive.moveTo(
                new Pose2d(-46, 49, Math.toRadians(90)),
                new Pose2d(-46, 17, Math.toRadians(90)),
                new Pose2d(-54, 17, Math.toRadians(90)),
                new Pose2d(-55, 49, Math.toRadians(90)),
                new Pose2d(-54, 17, Math.toRadians(90)),
                new Pose2d(-61.6, 17, Math.toRadians(90)),
                new Pose2d(-62.1, 48, Math.toRadians(90)));
    }



    protected void expGetYellowSamples(){
        addLogMarker("expGetYellowSamples");
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



    private Pose2d bluePose;

    protected void getSamplesFromSubmersibleBlue(double headingOffset){

        resetAndGoToBlueSubmersible(0,headingOffset);

        Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR,10,0.35),
                ()-> upper.colorOfSample() == 2||upper.colorOfSample() == 1,
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);},
                ()->{drive.moveTo(new Pose2d(drive.getCurrentPose().getX(),drive.getCurrentPose().getY()+2, Math.toRadians(183)),10);
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    tryAgainAtBlueSubmersible(15);}));
        drive.moveTo(new Pose2d(drive.getSimpleMovePosition().getX()+3.7, drive.getSimpleMovePosition().getY(), Math.toRadians(175)),100, ()->Action.buildSequence(update));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

    }

    protected void getSamplesFromSubmersibleRed(double headingOffset){

        resetAndGoToRedSubmersible(0,headingOffset);

        Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR,10,0.35),
                ()-> upper.colorOfSample() == 0||upper.colorOfSample() == 1,
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    upper.setSlidePower(0);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);},
                ()->{drive.moveTo(new Pose2d(drive.getCurrentPose().getX(),drive.getCurrentPose().getY()+2, Math.toRadians(0)),10);
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    tryAgainAtRedSubmersible(0);}));
        drive.moveTo(new Pose2d(drive.getSimpleMovePosition().getX()-2, drive.getSimpleMovePosition().getY(), Math.toRadians(-10)),100, ()->Action.buildSequence(update));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

    }

    protected void getSamplesFromSubmersibleBlueWithEmergencyAscent(double firstHeadingOffset, double secondHeadingOffset) {
        //THIS USES RED VALUES
        addLogMarker("getFromSubmersibleRed");

        drive.setSimpleMoveTolerance(3,3,Math.toRadians(6));
        drive.setSimpleMovePower(1);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 200));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        Action.add();(new SlideAction(upper, SSValues.SLIDE_MIN, 750));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 750));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        drive.moveTo(new Pose2d(40, 14+headingOffset, Math.toRadians(-135)), 0, () -> Action.buildSequence(update));
        drive.moveTo(new Pose2d(redBasket.getX()+8, -16, Math.toRadians(0)),30, ()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(-29, -6.3, Math.toRadians(0+firstHeadingOffset)), 30, () -> {//x=-26.5,y=-8-headingOffset
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT, 200));
                    Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.7),0));
                    Action.buildSequence(update);
                }
        );
        upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR,10,0.25),
                ()-> upper.colorOfSample() == 1|upper.colorOfSample() == 2,
                ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);
                },
                ()->{drive.moveTo(new Pose2d(-29, -6.3, Math.toRadians(0+secondHeadingOffset)), 30, () -> {
                            upper.setWristPos(SSValues.WRIST_DEFAULT);
                            upper.setIntake(SSValues.CONTINUOUS_SPIN);
                            upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                            Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 100));
                            Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
                            Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR,10,0.3),
                                    ()-> upper.colorOfSample() == 1|upper.colorOfSample() == 2,
                                    ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);
                                        delay(50);
                                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                                        delay(50);
                                    },
                                    ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);}));
                            Action.buildSequence(update);
                        }
                );}));

        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
    }

    protected void getSamplesFromSubmersibleRedWithEmergencyAscent(double firstHeadingOffset, double secondHeadingOffset) {
        addLogMarker("getFromSubmersibleRed");

        drive.setSimpleMoveTolerance(3,3,Math.toRadians(6));
        drive.setSimpleMovePower(1);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 200));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        Action.add();(new SlideAction(upper, SSValues.SLIDE_MIN, 750));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 750));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        drive.moveTo(new Pose2d(40, 14+headingOffset, Math.toRadians(-135)), 0, () -> Action.buildSequence(update));
        drive.moveTo(new Pose2d(redBasket.getX()+8, -16, Math.toRadians(0)),30, ()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(-29, -6.3, Math.toRadians(0+firstHeadingOffset)), 30, () -> {//x=-26.5,y=-8-headingOffset
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT, 200));
                    Action.buildSequence(update);
                }
        );
        upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR,10,0.25),
                ()-> upper.colorOfSample() == 1|upper.colorOfSample() == 2,
                ()->{
                    upper.switchSequence(SuperStructure.Sequences.RUN);
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);
                },
                ()->{drive.moveTo(new Pose2d(-29, -6.3, Math.toRadians(0+secondHeadingOffset)), 30, () -> {
                            upper.setWristPos(SSValues.WRIST_DEFAULT);
                            upper.setIntake(SSValues.CONTINUOUS_SPIN);
                            upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                            Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 100));
                            Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE));
                            Action.add(new FinishConditionActionGroup(new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR,10,0.3),
                                    ()-> upper.colorOfSample() == 1|upper.colorOfSample() == 2,
                                    ()->{
                                        upper.switchSequence(SuperStructure.Sequences.RUN);
                                        upper.setIntake(SSValues.CONTINUOUS_STOP);
                                        delay(50);
                                        upper.setGrabPos(SSValues.GRAB_CLOSED);
                                        delay(50);
                                    },
                                    ()->{upper.setIntake(SSValues.CONTINUOUS_STOP);}));
                            Action.buildSequence(update);
                        }
                );}));

        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        upper.setWristPos(SSValues.WRIST_DEFAULT);

    }


    private void resetAndGoToBlueSubmersible(int correctTime, double headingOffset) {
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(6));
        drive.setSimpleMovePower(1);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 200));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        Action.add();(new SlideAction(upper, SSValues.SLIDE_MIN, 750));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 750));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        drive.moveTo(new Pose2d(40, 14+headingOffset, Math.toRadians(-135)), 0, () -> Action.buildSequence(update));
        drive.moveTo(new Pose2d(blueBasket.getX(), 12, Math.toRadians(-180)), 0, () -> Action.buildSequence(update));
        drive.moveTo(new Pose2d(18, 12, Math.toRadians(180 + headingOffset)), 0, () -> {
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT, 300));
                    Action.add(new CustomWaitAction(() -> drive.setSimpleMovePower(0.4), 700));
                    Action.buildSequence(update);
                    upper.setWristPos(SSValues.WRIST_INTAKE);
                    upper.setIntake(SSValues.CONTINUOUS_SPIN);
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                }
        );
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    private void resetAndGoToRedSubmersible(int correctTime, double headingOffset){
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(6));
        drive.setSimpleMovePower(1);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 200));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
//        Action.add();(new SlideAction(upper, SSValues.SLIDE_MIN, 750));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 750));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        drive.moveTo(new Pose2d(40, 14+headingOffset, Math.toRadians(-135)), 0, () -> Action.buildSequence(update));
        drive.moveTo(new Pose2d(redBasket.getX()+8, -16, Math.toRadians(0)),0, ()->Action.buildSequence(update));
        drive.moveTo(new Pose2d(-28, -10, Math.toRadians(0+headingOffset)), 0, () -> {//x=-26.5,y=-8-headingOffset
                    Action.add(new GrabAction(upper, GrabAction.grabState.GRAB_DEFAULT, 400));
                    Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.5), 600));
                    Action.buildSequence(update);
                    upper.setWristPos(SSValues.WRIST_INTAKE);
                    upper.setIntake(SSValues.CONTINUOUS_SPIN);
                    upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
                }
        );
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void parkToBlueSumbersible(){
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(6));
        drive.setSimpleMovePower(1);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 200));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 750));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(blueBasket.getX(), 16, Math.toRadians(-180)), 0, () -> Action.buildSequence(update));

        Action.add(new ArmAction(upper, ArmAction.armState.ARM_HANG1));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_LONGER));
        Action.add(new WaitAction(100));
        Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.3), 0));

        drive.moveTo(new Pose2d(17, 16, Math.toRadians(180)), 0, () -> Action.buildSequence(update));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void parkToRedSumbersible(){
        drive.setSimpleMoveTolerance(3,3,Math.toRadians(6));
        drive.setSimpleMovePower(1);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 200));
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 750));
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_DEFAULT));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.moveTo(new Pose2d(redBasket.getX()+8, -16, Math.toRadians(0)),0, ()->Action.buildSequence(update));

        Action.add(new ArmAction(upper, ArmAction.armState.ARM_HANG1));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_LONGER));
        Action.add(new WaitAction(100));
        Action.add(new CustomWaitAction(()->drive.setSimpleMovePower(0.3), 0));

        drive.moveTo(new Pose2d(-27, -8, Math.toRadians(0)), 0, () -> Action.buildSequence(update));
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void tryAgainAtBlueSubmersible(double headingOffset){
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(1);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_SLIGHTLY_LONGER, 300));
        Action.buildSequence(update);

        SequentialActionGroup goAndGrab = new SequentialActionGroup(update,
                new DriveAction(drive, new Pose2d(22,drive.getCurrentPose().getY(), Math.toRadians(180+headingOffset)), 100, 300, update),
                new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50),
                new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR, 10, 0.35));

        CancellableFinishConditionActionGroup grabBlueFromSubmersibleAgain = new CancellableFinishConditionActionGroup(
                goAndGrab,
                () -> upper.colorOfSample() == 2 || upper.colorOfSample() == 1,
                () -> (System.currentTimeMillis() - startTime > 28 * 1000),
                () -> {
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);
                },
                () -> {
//                    accumulatedHeadingOffset = -(accumulatedHeadingOffset);
//                    tryAgainAtBlueSubmersible(accumulatedHeadingOffset);
                    tryAgainAtRedSubmersible(30);
                },
                () -> {
                    Action.clearActions();
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_LONGER));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_HANG1));
                    Action.buildSequence(update);
                    while (opModeIsActive()) {
                    }
                });

        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        //drive.moveTo(new Pose2d(22,drive.getCurrentPose().getY(), Math.toRadians(180+headingOffset)), 100, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        //upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.add(grabBlueFromSubmersibleAgain);
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
    }

    static double accumulatedHeadingOffset = 25;

    protected void tryAgainAtRedSubmersible(double headingOffset){
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(1);
        Action.buildSequence(update);

        SequentialActionGroup goAndGrab = new SequentialActionGroup(update,
                new DriveAction(drive, new Pose2d(-29.5,drive.getCurrentPose().getY(), Math.toRadians(0+headingOffset)), 40, 0, update),
                new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50),
                new SlideAction(upper, SlideAction.slideState.SLIDE_INTAKE_FAR, 10, 0.25));

        CancellableFinishConditionActionGroup grabRedFromSubmersibleAgain = new CancellableFinishConditionActionGroup(
                goAndGrab,
                () -> upper.colorOfSample() == 0 || upper.colorOfSample() == 1,
                () -> (System.currentTimeMillis() - startTime > 28 * 1000),
                () -> {
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    delay(50);
                    upper.setGrabPos(SSValues.GRAB_CLOSED);
                    delay(50);
                },
                () -> {
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                },
                () -> {
                    Action.clearActions();
                    upper.setIntake(SSValues.CONTINUOUS_STOP);
                    Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_LONGER));
                    Action.add(new ArmAction(upper, ArmAction.armState.ARM_HANG1));
                    Action.buildSequence(update);
                    while (opModeIsActive()) {
                    }
                });

        upper.setWristPos(SSValues.WRIST_ABOVE_SAMPLES);
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN));
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        //drive.moveTo(new Pose2d(22,drive.getCurrentPose().getY(), Math.toRadians(180+yOffset)), 100, ()->Action.buildSequence(update));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        //upper.setWristPos(SSValues.WRIST_INTAKE);
        upper.switchSequence(SuperStructure.Sequences.INTAKE_FAR);
        Action.add(grabRedFromSubmersibleAgain);
        Action.buildSequence(update);
        upper.setIntake(SSValues.CONTINUOUS_STOP);
        //////
    }


    protected Pose2d lastBlueBasketSample = new Pose2d(54.5, 47, Math.toRadians(-60));
    protected Pose2d lastBlueChamberSample = new Pose2d(-54.5, 47, Math.toRadians(-120));

    protected Pose2d lastRedBasketSample = new Pose2d(-54.8,-46.0,Math.toRadians(141));
    protected void moveAndIntakeLastBasketSampleBlue(){
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

    protected void moveAndIntakeLastChamberSampleBlue(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(5));
        drive.setSimpleMovePower(0.3);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 950));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
        drive.moveTo(lastBlueChamberSample, 150, ()->Action.buildSequence(update));
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

    protected void moveAndIntakeLastBasketSampleRed(){
        drive.setSimpleMoveTolerance(1,1,Math.toRadians(5));
        drive.setSimpleMovePower(0.3);
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        Action.add(new WristAction(upper, WristAction.wristState.WRIST_INTAKE, 50));
        Action.add(new SlideAction(upper, SlideAction.slideState.SLIDE_MIN, 950));
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_DOWN, 200));
//        moveToGetLastYellowSample();
        drive.moveTo(lastRedBasketSample, 150, ()->Action.buildSequence(update));
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

    protected void autoResetArmTest(){
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP));
        Action.buildSequence(update);
        upper.setArmTargetPosition(SSValues.ARM_DOWN-SSValues.AUTO_ARM_OFFSET);
    }

    public static  double testPIDx = 0, testPIDy = 0, testPIDheading = 90;
    public static  double targetPIDx = 0, targetPIDy = 40, targetPIDheading = 90;
    Pose2d[] poses = {new Pose2d(targetPIDx,targetPIDy,Math.toRadians(targetPIDheading)),new Pose2d(testPIDx,testPIDy,Math.toRadians(testPIDheading))};
    private Pose2d currentPose;
    private int poseCount = 0;
    protected void testAutoPID(){
        drive.setSimpleMovePower(1);
        if(!drive.isBusy()){
            if(poseCount < poses.length){
                currentPose = poses[poseCount];
                drive.moveTo(currentPose,200);
                poseCount++;
            }else{
                poseCount = 0;
            }
        }
    }

    protected void autoArmTest(){
        Action.add(new ArmAction(upper, ArmAction.armState.ARM_UP));
        Action.buildSequence(update);
    }


    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            update.run();
        }
    }

    long lastStepTime = 0;
    String previousStepCaption = "init";

    protected void addLogMarker(String newCaption){
        long currStepTime = System.currentTimeMillis() - lastStepTime;
        lastStepTime = System.currentTimeMillis();
        CentralLogController.log(previousStepCaption+" took "+currStepTime+" ms");
        previousStepCaption = newCaption;
    }
}