package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoRedBasket extends AutoMaster{
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto(new Pose2d(-44, -60, Math.toRadians(55)));

        while(opModeInInit()){

        }

        waitForStart();
        setStartTime();

        expFirstPutRedBasket();
        expResetAfterRedBasketAndMoveToIntake(0, -0.5, 10);

        expGetYellowSamples();

        putRedBasketFromGround(-2,0, 0);
        expResetAfterRedBasketAndMoveToIntake(-7.2, -4.5, 10);

        expGetYellowSamples();

        putRedBasketFromGround(0,-1, 0);

        moveAndIntakeLastBasketSampleRed();

        putRedBasketFromGround(0,-1, 0);


        getSamplesFromSubmersibleRedWithEmergencyAscent(0, 15);

        putRedBasketFromSubmersible(0,0, 0);


        getSamplesFromSubmersibleRedWithEmergencyAscent(30, -10);

        putRedBasketFromSubmersible(0,0 , 5);


        parkToRedSumbersible();




        while(opModeIsActive()){
            super.update.run();
        }


    }


}
