void PID_compute()
{
double d=0,e=0,f=0;//Formula constant
//perror=hpitch-s_kalAngleX;//Newest Pitch error in Kalman Filter & smoothing function
//rerror=hroll-s_kalAngleY;//Newest Roll error in Kalman Filter & smoothing function
//perror=hpitch-kalAngleX;//Newest Pitch error in Kalman Filter
//rerror=hroll-kalAngleY;//Newest Roll error in Kalman Filter
perror=hpitch-compAngleX;//Newest Pitch error in second order complementary filter
rerror=hroll-compAngleY;//Newest Roll error in second order complementary filter
yerror=hyaw-dir;// Yaw error

newtime=millis(); // Loads the new time for which the quad is away from stable position
dt=newtime-oldtime;
oldtime=newtime; // Remember the old time
//Serial.print("Time change: " );Serial.println(dt);

pOutput=(Kp*perror)+(Ki*perrsum*dt)+(Kd*((perror-perrlast)/dt)); // Pitch PID
//pOutput=(int)pOutput;
perrlast=perror;//Remembers the last error for next run
perrsum=perrsum+perror; //Adding up error overtime
//d=(Kd*((perror-perrlast)/dt));
//f=Kp*perror;
//e=(Ki*(perrsum*dt));
//Serial.print("Kp part: "); Serial.println(f);
//Serial.print("Kd part: "); Serial.println(d);
//Serial.print("Ki part: "); Serial.println(e);
//Serial.println(pOutput);

rOutput=(Kp*rerror)+(Ki*(rerrsum*dt))+(Kd*((rerror-rerrlast)/dt)); // Roll PID
//rOutput=(int)rOutput;
rerrlast=rerror;//Remembers the last error for next run
rerrsum=rerrsum+rerror; //Adding up error overtime
//Serial.println(rOutput);


yOutput=(zKp*yerror)+(zKi*(yerrsum*dt))+(zKd*((yerror-yerrlast)/dt)); // Yaw PID
//yOutput=(int)yOutput;
yerrlast=yerror;//Remembers the last error for next run
yerrsum=yerrsum+yerror; //Adding up error overtime
//Serial.println(yOutput);

if (gthrot>takeoff) // Will ensure that PID doesnt start up the motors
 {
   //Equation 1.1 All Axis Mix with damper. all response correct
   pwm_motA1=(0.7)*pOutput-yOutput+gthrot+(0.7)*rOutput;
   pwm_motA2=(-0.7)*rOutput+yOutput+gthrot+(0.7)*pOutput;
   pwm_motA3=(0.7)*rOutput+yOutput+gthrot-(0.7)*pOutput;
   pwm_motA4=(-0.7)*pOutput-yOutput+gthrot-(0.7)*rOutput;
   //Motors cannot shutdown here. When the global value will fall it will automatically help shuttind down the motors
    if (pwm_motA1<mot_min) pwm_motA1=mot_min;
    if (pwm_motA2<mot_min) pwm_motA2=mot_min;
    if (pwm_motA3<mot_min) pwm_motA3=mot_min;
    if (pwm_motA4<mot_min) pwm_motA4=mot_min;
     
 }
else
 {
  perror=0;
  rerror=0;
  yerror=0;
  perrsum=0;
  perrlast=0;
  rerrsum=0;
  rerrlast=0;
  yerrsum=0;
  yerrlast=0;
  pwm_motA1=gthrot;
  pwm_motA2=gthrot;
  pwm_motA3=gthrot;
  pwm_motA4=gthrot;
 }
}
//End of Stabilize()

void quad_home()
{
for (int i=0; i<=samplerate-1; i++)
 {
   getPR();
   
 } 
//hpitch=kalAngleX;
//hroll=kalAngleY;
hpitch=compAngleX;
hroll=compAngleY;

// High pass filter for compass
for (int k1=0; k1<=samplerate-1; k1++)
{
  low_last=low_sum;//Keep record of previous sum
  get_yaw();// Get yaw
  low_dir=dir;//Load newest direction value with noise into the variable
  low_sum=low_sum+low_dir;//Update new sum
}
hyaw=(low_sum-low_last);

#ifdef Debug_Home
  Serial.print("Desired Pitch: "); Serial.print(hpitch);
  Serial.println();
  Serial.print("Desired Roll: "); Serial.print(hroll);
  Serial.println();
  Serial.print("Desired Yaw: "); Serial.print(hyaw);
  Serial.println();
  Serial.print("Initial Height: "); Serial.print(halt); Serial.print("m");
  Serial.println();
  #endif
pcrr=180.0-hpitch; // Negative value
rcrr=180.0-hroll;
pcrr=pcrr*(-1);
}

