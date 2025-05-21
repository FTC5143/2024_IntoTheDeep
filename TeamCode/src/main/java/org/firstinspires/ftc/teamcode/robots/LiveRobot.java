package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.live.Arm;
import org.firstinspires.ftc.teamcode.components.live.DriveTrain;
import org.firstinspires.ftc.teamcode.components.live.Intake;
import org.firstinspires.ftc.teamcode.components.live.LEDControl;
import org.firstinspires.ftc.teamcode.components.live.Lift;
import org.firstinspires.ftc.teamcode.components.live.Reach;

public class LiveRobot extends Robot {
//    public OCVWebCamera     web_camera;

    public DriveTrain       drive_train;
    public Lift             lift;
    public Reach            reach;
    public Intake           intake;
    public Arm              arm;

    public LEDControl       led_control;

    FtcDashboard            dashboard;

    {
        name = "BORIS";
    }

    public LiveRobot(LinearOpMode opmode) {
        super(opmode);
 //       web_camera      = new OCVWebCamera(this);

        drive_train     = new DriveTrain(this);
        lift            = new Lift(this);
        reach           = new Reach(this);
        intake          = new Intake(this);
        arm             = new Arm(this);

        led_control     = new LEDControl(this);
        dashboard       = FtcDashboard.getInstance();
    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public void updateTelemetry() {
        super.updateTelemetry();
    }
}