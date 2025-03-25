package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.actioncore.Action;

//@Photon
@TeleOp(name = "16093 Blue Basket Double TeleOp")
public class TeleOp16093BlueBasket extends TeleOpMaster {
    @Override
    public void runOpMode() throws InterruptedException {

        initTeleOp(()->(upper.alphaAdjustedSampleColor() == 1||upper.alphaAdjustedSampleColor() == 2),180);

        // Wait until play button is pressed

        waitForStart();


        // Set intake to default stop position and initialize operation mode
//        upper.startIntake();

        // Main control loop while op mode is active
        while (opModeIsActive() && !isStopRequested()) {
            update.run();
            Action.buildSequence(update);

        }
    }

}