package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "AutoChamber")
public class AutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(-9.5  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();

        VexpPushTwoBlueSamples();


//        expResetChamberAndMoveToIntake(0, 0.5, 0, true);
//        expGetYellowSamples();
//        throwBehind();
//
////        expResetChamberAndMoveToIntake(-8.7, 1.8, 0, false);
//        turnTo(-120, 30);
//        expGetYellowSamples();
//        throwBehind();
//
//        turnTo(-140, 30);
//        throwBehind();



        intakeSpecimenFromBlueWall(0,1);
        blueChamberPlaceFromWall(8,-0.4);

        intakeSpecimenFromBlueWall(0,0);
        blueChamberPlaceFromWall(6,0);

        intakeSpecimenFromBlueWall(0,-0);
        blueChamberPlaceFromWall(4,0);

        intakeSpecimenFromBlueWall(0,-0);
        blueChamberPlaceFromWall(2,0);

//        parkFromBlueChamber();
        intakeSpecimenFromBlueWall(0,-0);
        blueChamberPlaceFromWall(1,0);


        while(opModeIsActive()){
            super.update.run();
        }

    }


}