package org.firstinspires.ftc.teamcode.autoTestCodes;

public class tickToDest1 {

    /* basically this method will take the motor tpr (ticks per revolution) and the circumference of the wheel,
    * with these two we can calculate how many ticks it takes to travel cm,
    * meaning - if the wheel's circumference = 12cm, we can divide the tpr by the wheel circumference to -
    * how many ticks traveled per cm.
    * after that, we will multiply this amount of ticks by how many cms we wanna travel. and thats it! */

    public int ticksCalculator(int Motor_tpr, double circumference, int distance) throws InterruptedException {

//        Motor_tpr = 0; // with gears
//        circumference = 0;
        double ticksPerCM = Motor_tpr / circumference;
//        distance = 100; // in cm
        int TickToDest = (int) (distance * ticksPerCM);

        return TickToDest;

    }

}
