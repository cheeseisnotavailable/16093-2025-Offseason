package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoMaster;

@Autonomous
public class TestAutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(-9.5  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();

        intakeSpecimenFromBlueWall(-6,0);
        blueChamberPlaceFromWall(14,0);

        intakeSpecimenFromBlueWall(-6,0);
        blueChamberPlaceFromWall(12,0);

        intakeSpecimenFromBlueWall(-5,-0);
        blueChamberPlaceFromWall(10,0);

        intakeSpecimenFromBlueWall(-5,-0);
        blueChamberPlaceFromWall(8,0);

        intakeSpecimenFromBlueWall(-5,-0.5);
        blueChamberPlaceFromWall(6.5,0);

//        intakeSpecimenFromWall(-2);

//        intakeSpecimenFromGround(-1,-0);
//        newBlueChamberPlace(12,-0);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromGround(-1,-0);
//        newBlueChamberPlace(11,-0);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromGround(-1,-1);
//        newBlueChamberPlace(8,-0);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromGround(-1,-1);
//        newBlueChamberPlace(5,-0);
//        newResetFromHighChamber();
//
//        intakeSpecimenFromGround(-1,-1);
//        newBlueChamberPlace(2,-0);
//        newResetFromHighChamber();

        while(opModeIsActive()){
            super.update.run();
        }

    }


}