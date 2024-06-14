package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Subsystem;

@Config
public class Launcher extends Subsystem {
    private final Servo servo;
    public static double LAUNCH_POS = 0.72;
    public static double IDLE_POS = 0.5;

    public Launcher(final HardwareMap hwMap, final MultipleTelemetry telemetry) {
        super(telemetry);

        this.servo = hwMap.get(Servo.class, "launcher");
        this.servo.setDirection(Servo.Direction.REVERSE);
    }

    public void launch() {
        this.servo.setPosition(Launcher.LAUNCH_POS);
    }

    public void idle() {
        this.servo.setPosition(Launcher.IDLE_POS);
    }
}