package org.firstinspires.ftc.teamcode.actions.actioncore;

public class BranchActionPair {
    public Action pairedAction;
    public Runnable pairedRunnable;

    public BranchActionPair(Action pairedAction, Runnable pairedRunnable){
        this.pairedAction = pairedAction;
        this.pairedRunnable = pairedRunnable;
    }
}
