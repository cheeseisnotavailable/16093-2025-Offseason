package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.SuperStructure;
import org.firstinspires.ftc.teamcode.actions.actioncore.ServoAction;
import org.firstinspires.ftc.teamcode.references.SSValues;

public class WristAction extends ServoAction {

    public WristAction(SuperStructure upper, wristState state){
        super(upper,0);
        this.pos = returnWristPosition(state);
    }

    public WristAction(SuperStructure upper, wristState state, int waitTime){
        super(upper,0,waitTime);
        this.pos = returnWristPosition(state);
    }

    public String returnType(){
        return "WristAction";
    }

    public void actuate() {
        upper.setWristPos(pos);
    }

    public enum wristState{
        WRIST_DEFAULT,
        WRIST_INTERMEDIATE,
        WRIST_HIGH_CHAMBER,
        WRIST_RELEASE_AUTO,
        WRIST_RELEASE_TELEOP,
        WRIST_RELEASE_EXTRA,
        WRIST_ABOVE_SAMPLES,
        WRIST_INTAKE,
        WRIST_INTAKE_SPECIMEN_DONTUSETHIS,
        WRIST_HIGH_CHAMBER_RESET,
        WRIST_INTAKE_WALL_SPECIMEN
    }

    public double returnWristPosition(wristState state){
        switch (state) {
            case WRIST_DEFAULT:
                return SSValues.WRIST_DEFAULT;
            case WRIST_INTERMEDIATE:
                return SSValues.WRIST_INTERMEDIATE;
            case WRIST_HIGH_CHAMBER:
                return SSValues.WRIST_HIGH_CHAMBER;
            case WRIST_RELEASE_AUTO:
                return SSValues.WRIST_RELEASE_AUTO;
            case WRIST_RELEASE_TELEOP:
                return SSValues.WRIST_RELEASE_TELEOP;
            case WRIST_RELEASE_EXTRA:
                return SSValues.WRIST_ABOVE_SAMPLES;
            case WRIST_ABOVE_SAMPLES:
                return SSValues.WRIST_ABOVE_SAMPLES;
            case WRIST_INTAKE:
                return SSValues.WRIST_INTAKE;
            case WRIST_INTAKE_SPECIMEN_DONTUSETHIS:
                return SSValues.WRIST_INTAKE_SPECIMEN_DONTUSETHIS;
            case WRIST_HIGH_CHAMBER_RESET:
                return SSValues.WRIST_HIGH_CHAMBER_RESET;
            case WRIST_INTAKE_WALL_SPECIMEN:
                return SSValues.WRIST_INTAKE_WALL_SPECIMEN;
        }
        return 0;
    }

}
