void motor_setup()
{
pinMode (motA1, OUTPUT); // setting this as output
pinMode (motA2, OUTPUT); // setting this as output
pinMode (motA3, OUTPUT); // setting this as output
pinMode (motA4, OUTPUT); // setting this as output
pwm_motA1=gthrot;
pwm_motA2=gthrot;
pwm_motA3=gthrot;
pwm_motA4=gthrot;
//Will ensure any accidental starting of motors doesn't happen
analogWrite(motA1, pwm_motA1);
analogWrite(motA2, pwm_motA2);
analogWrite(motA3, pwm_motA3);
analogWrite(motA4, pwm_motA4);
delay(5);
}
//end of motor_setup()


void motor_update()
{
// Safety against overshoot
if (pwm_motA1>=mot_max) pwm_motA1=mot_max;
if (pwm_motA2>=mot_max) pwm_motA2=mot_max;
if (pwm_motA3>=mot_max) pwm_motA3=mot_max;
if (pwm_motA4>=mot_max) pwm_motA4=mot_max;

#ifdef Debug_Motor
Serial.print("M1: "); Serial.println(pwm_motA1);
Serial.print("M2: "); Serial.println(pwm_motA2);
Serial.print("M3: "); Serial.println(pwm_motA3);
Serial.print("M4: "); Serial.println(pwm_motA4);
#endif
analogWrite(motA1, pwm_motA1);
analogWrite(motA2, pwm_motA2);
analogWrite(motA3, pwm_motA3);
analogWrite(motA4, pwm_motA4);
delay(1); 
}// End of motor update

void throttle()
{
if (data[1]==0xA3&&data[2]==1&&t_p_flag==false)// Throttle Up
        {
          gthrot=gthrot+dthrot;
          pwm_motA1=gthrot; 
          pwm_motA2=gthrot; 
          pwm_motA3=gthrot; 
          pwm_motA4=gthrot;
          t_p_flag=true;
          t_m_flag=false;
          motor_update();
        }
        else if (data[1]==0xA4&&data[2]==0&&t_m_flag==false) //Throttle down
          {
            gthrot=gthrot-dthrot;
            pwm_motA1=gthrot; 
            pwm_motA2=gthrot; 
            pwm_motA3=gthrot; 
            pwm_motA4=gthrot;
            t_p_flag=false;
            t_m_flag=true;
            motor_update();
          }
          else
          {
            // DO NOT PUT LINES TO KEEP BOTH FLAGS CLEAR HERE IT MAKES MEGA RECEIVE ONE BUTTON PUSH AS MANY!!!
          }
    
}
//End of throttle()


void my_yaw()
{
if (dyaw==true)// Turn quad clockwise
    {
      pwm_motA1=gthrot-pwm_dyaw;
      pwm_motA2=gthrot+pwm_dyaw;
      pwm_motA3=gthrot+pwm_dyaw;
      pwm_motA4=gthrot-pwm_dyaw;
      //Motors cannot shutdown here
      if (pwm_motA1<mot_min) pwm_motA1=mot_min;
      if (pwm_motA2<mot_min) pwm_motA2=mot_min;
      if (pwm_motA3<mot_min) pwm_motA3=mot_min;
      if (pwm_motA4<mot_min) pwm_motA4=mot_min;
      hyaw=dir;//Callibrating yaw 
      //motor_update();

    }
    else if (dyaw==false) // Turn quad anticlockwise
      {
        pwm_motA1=gthrot+pwm_dyaw;
        pwm_motA2=gthrot-pwm_dyaw;
        pwm_motA3=gthrot-pwm_dyaw;
        pwm_motA4=gthrot+pwm_dyaw;
        //Motors cannot shutdown here
        if (pwm_motA1<mot_min) pwm_motA1=mot_min;
        if (pwm_motA2<mot_min) pwm_motA2=mot_min;
        if (pwm_motA3<mot_min) pwm_motA3=mot_min;
        if (pwm_motA4<mot_min) pwm_motA4=mot_min;
        hyaw=dir;//Callibrating yaw
        //motor_update();
       }
      else
      {
        // THIS WILL NEVER BE CALLED
      }

}
// end of my_yaw()
int find_xn()
{
// Negative range

if(xstick>350&&xstick<=450) 
  {
    x1=1;
  }
  else if (xstick>150&&xstick<=349)
  {
    x1=2;
  }
  else
  {
    x1=4;
  }

//

return x1;
}
int find_xp()
{
  // Positive range
  if(xstick>511&&xstick<=749) 
  {
    x2=1;
  }
  else if (xstick>750&&xstick<=900)
  {
    x2=2;
  }
  else
  {
    x2=4;
  }
//
return x2;
}

int find_yn()
{
// Negative range

if(ystick>350&&ystick<=450) 
  {
    y1=1;
  }
  else if (ystick>150&&ystick<=349)
  {
    y1=2;
  }
  else
  {
    y1=4;
  }

//

return y1;
}
int find_yp()
{
  // Positive range
  if(ystick>511&&ystick<=749) 
  {
    y2=1;
  }
  else if (ystick>750&&ystick<=900)
  {
    y2=2;
  }
  else
  {
    y2=4;
  }
//
return y2;
}
