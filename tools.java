package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.EventLoop;

public class tools
{
    public static class math
    {
        public static boolean isNumberinRange(double input,double reference,double errorRange)
        {
            double minus = Math.abs(input-reference);
            if(minus <= errorRange) return true;
            else return false;
        }

        public static double limitNumber(double input,double min,double max)
        {
            if(input >= max) return max;
            else if(input <= min) return min;
            else return input;
        }

        public static int limitNumber(int input,int min,int max)
        {
            return (int)limitNumber((double)input,min,max);
        }
    }
}
