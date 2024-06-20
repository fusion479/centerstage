package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.arcrobotics.ftclib.command.CommandScheduler;

public class RunScheduler extends Thread {
    public void run() {
        while (true) {
            try {
                CommandScheduler.getInstance().run();
                Thread.sleep(50);
            } catch (Exception e) {
                CommandScheduler.getInstance().cancelAll();
                CommandScheduler.getInstance().disable();
                CommandScheduler.getInstance().reset();
            }
        }
    }
}
