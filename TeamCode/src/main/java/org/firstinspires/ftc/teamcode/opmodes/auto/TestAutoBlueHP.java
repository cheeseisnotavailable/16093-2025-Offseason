package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoMaster;

@Autonomous
@Disabled
public class TestAutoBlueHP extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(-9.5  ,62.3 ,Math.toRadians(90)));

        while(opModeInInit()){

        }

        waitForStart();
        newFirstMoveToBlueChamberPlace();

        intakeSpecimenFromBlueWall(-6,0);
        blueChamberPlaceFromWall(13,0);

        intakeSpecimenFromBlueWall(-6,0);
        blueChamberPlaceFromWall(11,0);

        intakeSpecimenFromBlueWall(-5,-0);
        blueChamberPlaceFromWall(9,0);

        intakeSpecimenFromBlueWall(-5,-0);
        blueChamberPlaceFromWall(7,0);

        intakeSpecimenFromBlueWall(-5,-0.5);
        blueChamberPlaceFromWall(5,0);

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