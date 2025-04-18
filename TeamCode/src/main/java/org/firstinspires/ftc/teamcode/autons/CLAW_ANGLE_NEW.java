package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem_Constants;

public class CLAW_ANGLE_NEW {
    private Servo Claw_Angle;
    final double claw_AngleScale0 = Subsystem_Constants.claw_AngleScale0;
    final double claw_AngleScale1 = Subsystem_Constants.claw_AngleScale1;
    final double claw_AngleForward = Subsystem_Constants.claw_AngleForward;
    final double claw_AngleBackward = Subsystem_Constants.claw_AngleBackward;
    final double claw_AngleLeft = Subsystem_Constants.claw_AngleLeft;
    public CLAW_ANGLE_NEW(HardwareMap hardwareMap) {
        Claw_Angle = hardwareMap.get(Servo.class, "Claw_Angle");
        Claw_Angle.scaleRange(claw_AngleScale0, claw_AngleScale1);
    }

    public class MoveClawAngle implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        double pos;
        public MoveClawAngle(double pos){
            start = false;
            runTime = 0;
            this.pos = pos;
        }
        public MoveClawAngle(double pos, double runt){
            start = false;
            runTime = runt;
            this.pos = pos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            } else {
                Claw_Angle.setPosition(pos);
                return false;
            }
        }
    }
    public Action forward() {
        return new MoveClawAngle(claw_AngleForward);
    }
    public Action forward(double runt) {
        return new MoveClawAngle(claw_AngleForward,runt);
    }
    public Action backward() {
        return new MoveClawAngle(claw_AngleBackward);
    }
    public Action backward(double runt) {
        return new MoveClawAngle(claw_AngleBackward,runt);
    }
    public Action sub(){return new MoveClawAngle(claw_AngleLeft);}
    public Action sub(double runt) {
        return new MoveClawAngle(claw_AngleLeft,runt);
    }
}
