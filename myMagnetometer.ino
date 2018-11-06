void mymag_setup()
{
    
    // This turns on the MPU's auxillary bus
    //Wire library was already defined in the main sketch
    accelgyro.setI2CMasterModeEnabled(false);
    accelgyro.setI2CBypassEnabled(true) ;
    accelgyro.setSleepEnabled(false);
    mag.initialize();
}
//End of mymag_setup()

void get_yaw()
{
mag.getHeading(&mx, &my, &mz);
// To calculate heading in degrees. 
// 0 degree indicates North
float heading = atan2(my, mx);
if(heading < 0)
heading += 2 * M_PI;
dir=heading * 180/M_PI;
}
// End of get_yaw()


