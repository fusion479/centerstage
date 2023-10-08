package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
  public static double intakePos = 0.69;
  public static double scorePos = 0.69;

  public static double open = 0.69;
  public static double close = 0.69;
  public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "clawDep");
        left = hwMap.get(Servo.class, "Left");
        right = hwMap.get(Servo.class, "Right");
  }
  
}

public void collect {
  left.setPosition(intakePos);
  right.setPosition(intakePos - 1);
}

public void score {
  left.setPosition(scorePos);
  right.setPosition(scorePos - 1);
}

public void open {
  claw.setPosition(open)
}

public void close {
  claw.setPosition(close)
}

// i hope i know what im doing
