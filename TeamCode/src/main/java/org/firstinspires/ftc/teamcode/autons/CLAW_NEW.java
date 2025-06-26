package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem_Constants;

public class CLAW_NEW {
    private Servo Claw;
    final double clawScale0 = Subsystem_Constants.clawScale0;
    final double clawScale1 = Subsystem_Constants.clawScale1;
    final double closeClaw = Subsystem_Constants.closeClaw;
    final double openClaw = Subsystem_Constants.openClaw;
    final double openMore = Subsystem_Constants.openMore;
    final double openLess = Subsystem_Constants.openLess;
    public CLAW_NEW(HardwareMap hardwareMap) {
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.scaleRange(clawScale0,clawScale1);
    }


    public class MoveClaw implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        double pos;
        public MoveClaw(double pos){
            start = false;
            runTime = 0;
            this.pos = pos;
        }
        public MoveClaw(double pos, double runt){
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
                Claw.setPosition(pos); //0.28 //0.65
                return false;
            }
        }
    }
    public Action closeClaw() {
        return new MoveClaw(closeClaw);
    }
    public Action closeClaw(double runt) {
        return new MoveClaw(closeClaw, runt);
    }
    public Action openClaw() {
        return new MoveClaw(openClaw);
    }
    public Action openClaw(double runt) {
        return new MoveClaw(openClaw, runt);
    }
    public Action openClawMore() {
        return new MoveClaw(openMore);
    }
    public Action openClawMore(double runt) {
        return new MoveClaw(openMore, runt);
    }
    public Action openClawLess() {
        return new MoveClaw(openLess);
    }
    public Action openClawLess(double runt) {
        return new MoveClaw(openLess, runt);
    }
}
