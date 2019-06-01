package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GamepadTest", group = "Test")
public class Test_gamepad extends OpMode
{
//    private ElapsedTime runtime = new ElapsedTime();//计时

    public void init()
    {
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start()
    {
        telemetry.clearAll();
        //   runtime.reset();
    }

    @Override
    public void loop()
    {
        telemetry.addData("LY:\t",gamepad1.left_stick_y);
        telemetry.addData("LX:\t",gamepad1.left_stick_x);
        telemetry.addData("RX:\t",gamepad1.right_stick_x);
        telemetry.addData("RY:\t",gamepad1.right_stick_y);
        telemetry.addData("X:\t",gamepad1.x);
        telemetry.addData("Y:\t",gamepad1.y);
        telemetry.addData("B:\t",gamepad1.b);
        telemetry.addData("A:\t",gamepad1.a);
        telemetry.addData("LB:\t",gamepad1.left_bumper);
        telemetry.addData("RB:\t",gamepad1.right_bumper);
        telemetry.addData("LT:\t",gamepad1.left_trigger);
        telemetry.addData("RT:\t",gamepad1.right_trigger);
        telemetry.addData("UP:\t",gamepad1.dpad_up);
        telemetry.addData("DOWN:\t",gamepad1.dpad_down);
        telemetry.addData("LEFT:\t",gamepad1.dpad_left);
        telemetry.addData("RIGHT:\t",gamepad1.dpad_right);
        telemetry.addData("LButton:\t",gamepad1.left_stick_button);
        telemetry.addData("RButton:\t",gamepad1.right_stick_button);
        telemetry.addData("BACK:\t",gamepad1.back);
        telemetry.addData("START:\t",gamepad1.start);
        telemetry.update();
    }
}
