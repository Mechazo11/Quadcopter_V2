void myVaro_setup()
{
initial(ADDRESS);
}
// end of myVaro_setup()


void get_height()
{
D1 = getVal(ADDRESS, 0x48); // Pressure raw
D2 = getVal(ADDRESS, 0x58);// Temperature raw
dT   = D2 - ((uint32_t)C[5] << 8);
OFF  = ((int64_t)C[2] << 16) + ((dT * C[4]) >> 7);
SENS = ((int32_t)C[1] << 15) + ((dT * C[3]) >> 8);
TEMP = (int64_t)dT * (int64_t)C[6] / 8388608 + 2000;

  if(TEMP < 2000) // if temperature lower than 20 Celsius 
  {
    int32_t T1    = 0;
    int64_t OFF1  = 0;
    int64_t SENS1 = 0;

    T1    = pow(dT, 2) / 2147483648;
    OFF1  = 5 * pow((TEMP - 2000), 2) / 2;
    SENS1 = 5 * pow((TEMP - 2000), 2) / 4;
    
    if(TEMP < -1500) // if temperature lower than -15 Celsius 
      {
        OFF1  = OFF1 + 7 * pow((TEMP + 1500), 2); 
        SENS1 = SENS1 + 11 * pow((TEMP + 1500), 2) / 2;
      }
    
    TEMP -= T1;
    OFF -= OFF1; 
    SENS -= SENS1;
  }
Temperature = (float)TEMP / 100; 
P  = ((int64_t)D1 * SENS / 2097152 - OFF) / 32768;
Pressure = (float)P / 100;
h=getAltitude(Pressure, Temperature); // Get height into a global variable
h_cm=h*100; // Converting to centimeter
}
// End of get_height()

float getAltitude(float Pressure, float Temperature)
{
return ((pow((sea_press / Pressure), 1/5.257) - 1.0) * (Temperature + 273.15)) / 0.0065;
}
// End of getAltitude()

long getVal(int address, byte code)
{
  unsigned long ret = 0;
  Wire.beginTransmission(address);
  Wire.write(code);
  Wire.endTransmission();
  delay(10);
  // start read sequence
  Wire.beginTransmission(address);
  Wire.write((byte) 0x00);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom(address, (int)3);
  if (Wire.available() >= 3)
  {
    ret = Wire.read() * (unsigned long)65536 + Wire.read() * (unsigned long)256 + Wire.read();
  }
  else {
    ret = -1;
  }
  Wire.endTransmission();
  return ret;
}
// End if getVal()

void initial(uint8_t address)
{
  Wire.beginTransmission(address);
  Wire.write(0x1E); // reset
  Wire.endTransmission();
  delay(10);

   for (int i=0; i<6  ; i++) 
   {

    Wire.beginTransmission(address);
    Wire.write(0xA2 + (i * 2));
    Wire.endTransmission();

    Wire.beginTransmission(address);
    Wire.requestFrom(address, (uint8_t) 6);
    delay(1);
    if(Wire.available())
    {
       C[i+1] = Wire.read() << 8 | Wire.read();
    }
   }

}
// End of initial


