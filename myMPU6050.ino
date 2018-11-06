void mpu6050_setup()
{
  //TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 0; // 8kHz/(0+1) = 8000Hz, max out
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x03; // Set Accelerometer Full Scale Range to ±2g, 16g when 0x03
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100);// To stabilize the sensor
  
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

  //accYangle = (atan2(accZ, accX) + PI) * RAD_TO_DEG;
  //accXangle = (atan2(accZ, accY) + PI) * RAD_TO_DEG;

  
  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  timer = micros();
}
// End of mpu6050_setup()
void getPR()
{
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  //tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  
  //accYangle = (atan2(accZ, accX) + PI) * RAD_TO_DEG;
  //accXangle = (atan2(accZ, accY) + PI) * RAD_TO_DEG;
  
  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);
  
  // Calculate gyro angle using the unbiased rate
  //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); 
  //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);

  // Calculate gyro angle without any filter
  //gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); 
  //gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);
  
  // Calculate the angle using a Kalman filter
  //kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); 
  //kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  //timer = micros();
  
  // Calculate the angle using a First Order Complimentary filter
  //compAngleX = (a * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + ((1-a) * accXangle);
  //compAngleY = (a * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + ((1-a) * accYangle);
  //timer = micros();
  
   //First Order Complimentary filter with timeConstant 
  compAngleX = a * (compAngleX + (gyroXrate * filter_diff)) + (1-a) * accXangle;
  compAngleY = a * (compAngleY + (gyroYrate * filter_diff)) + (1-a) * accYangle;
  
  
  
  //Calculate the angle using a Second Order Complimentary Filter
  filter_xterm[0] = (accXangle - compAngleX) * timeConstant * timeConstant;
  filter_yterm[0] = (accYangle - compAngleY) * timeConstant * timeConstant;
  filter_xterm[2] = (filter_diff * filter_xterm[0]) + filter_xterm[2];
  filter_yterm[2] = (filter_diff * filter_yterm[0]) + filter_yterm[2];
  filter_xterm[1] = filter_xterm[2] + (accXangle - compAngleX) * 2 * timeConstant + gyroXrate;
  filter_yterm[1] = filter_yterm[2] + (accYangle - compAngleY) * 2 * timeConstant + gyroYrate;
  compAngleX = (filter_diff * filter_xterm[1]) + compAngleX;
  compAngleY = (filter_diff * filter_yterm[1]) + compAngleY;
  
}
// End of get_PR()

