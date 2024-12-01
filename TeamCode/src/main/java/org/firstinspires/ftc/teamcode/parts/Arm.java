package org.firstinspires.ftc.teamcode.parts;

import org.firstinspires.ftc.teamcode.Parts;

public interface Arm {
    public void pivot(int ticks);
    public void extend(int ticks);
}

class directGear implements Arm {
    public void pivot(int ticks) {
        Parts.setArm = ticks;
        Parts.arm.setTargetPosition(ticks);
    }
    public void extend(int ticks) {
        Parts.setSlide = ticks;
        Parts.slide.setTargetPosition(ticks);
    }
}
