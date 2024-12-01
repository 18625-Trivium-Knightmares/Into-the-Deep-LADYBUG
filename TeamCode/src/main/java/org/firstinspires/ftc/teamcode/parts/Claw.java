package org.firstinspires.ftc.teamcode.parts;

import org.firstinspires.ftc.teamcode.Parts;

public interface Claw {
    public void grabs();
    public void drops();
}

class gearClaw implements Claw {
    public void grabs() {
        Parts.claw.setPosition(1.00);
    }
    public void drops() {
        Parts.claw.setPosition(0.77);
    }
}