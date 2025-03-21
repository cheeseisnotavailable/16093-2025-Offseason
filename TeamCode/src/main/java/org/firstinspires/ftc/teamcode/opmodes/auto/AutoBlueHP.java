package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
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



        intakeSpecimenFromBlueWall(-6,0);
        blueChamberPlaceFromWall(12,0);

        intakeSpecimenFromBlueWall(-5,-0);
        blueChamberPlaceFromWall(10,0);

        intakeSpecimenFromBlueWall(-5,-0);
        blueChamberPlaceFromWall(8,0);

        intakeSpecimenFromBlueWall(-5,-0.5);
        blueChamberPlaceFromWall(6.5,0);

        parkFromBlueChamber();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}