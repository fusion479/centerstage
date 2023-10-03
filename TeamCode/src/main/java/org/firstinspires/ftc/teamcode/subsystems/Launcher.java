package org.firstinspires.ftc.teamcode.subsystems;

public class Launcher {

    // No clue what I'm doing ngl

    public static double LAUNCH = 0;
    public static double IDLE = 0;

    public void init(HardwareMap hwMap) {
        launcher = hwMap.get(Servo.class, "launcher");
    }

    public void launch() {
        launcher.setPosition(LAUNCH);
    }

    public void reset() {
        launcher.setPosition(IDLE);
    }

}
