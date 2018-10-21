#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter



SoftwareSerial mySerial(2, 3); // RX, TX
//-----------------------------DEFINE--------------------------------------

#define Tx 3
#define Rx 2
#define ALPHA 0.1
#define MULTIPLIER 6.67
#define FL_MOTOR 5
#define FR_MOTOR 9
#define BR_MOTOR 10
#define BL_MOTOR 11
#define STOPPED  0
#define STARTING 1
#define STARTED  2
#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf


//----------------------------VARIABLE-----------------------------------
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

float motorBattery;
int actSpeed[4];
int targetSpeed[4];
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
float instruction[4];
float delta_err[3]      = {0, 0, 0};
float Kp[3]      = {.53, .53, .53};
float Kd[3]      = {0.12, 0.12,0.12};
float Ki[3]      = {0.02, 0.02, 0.02};
char data;
float yaw_pid,roll_pid,pitch_pid;
double Pitchgap;
double Rollgap;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;

//------------ACCELEROMETER-------------------

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll,angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;

//-------------------------------FUNCTION--------------------------------


float smoothBattery (float prevEntry, float newEntry, float alpha) {
  return (1-alpha) * prevEntry + alpha * newEntry;
}


void runIndividual (int* targetSpeed) {
  analogWrite(FL_MOTOR, targetSpeed[0]);
  analogWrite(FR_MOTOR, targetSpeed[1]);
  analogWrite(BL_MOTOR, targetSpeed[2]);
  analogWrite(BR_MOTOR, targetSpeed[3]);
}

void resetGyroAngles()
{
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
}

/**
 * Reset motors' pulse length to 1000µs to totally stop them.
 */
void stopAll()
{
  analogWrite(FL_MOTOR,0);
  analogWrite(FR_MOTOR, 0);
  analogWrite(BR_MOTOR, 0);
  analogWrite(BL_MOTOR, 0);
}

//-------------PID----------------------

void calculateErrors() {
    errors[PITCH] = instruction[PITCH] - kalAngleY;
    errors[ROLL]  = instruction[ROLL]  - kalAngleX;
}


/**
 * Reset all PID controller's variables.
 */
void resetPidController()
{
    Pitchgap=0;
    Rollgap=0;
}

void pid(int* actSpeed){


        targetSpeed[0] = instruction[YAW];
    targetSpeed[1] = instruction[PITCH];
    targetSpeed[2] = instruction[ROLL];
    targetSpeed[3] = instruction[THROTTLE];
     //error_sum[YAW]   += errors[YAW];
        error_sum[PITCH] += errors[PITCH];
        error_sum[ROLL]  += errors[ROLL];

        // Calculate error delta : Derivative coefficients
       // delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
        delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
        delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

        // Save current error as previous_error for next time
       // previous_error[YAW]   = errors[YAW];
        previous_error[PITCH] = errors[PITCH];
        previous_error[ROLL]  = errors[ROLL];

        // PID = e.Kp + ∫e.Ki + Δe.Kd
        //yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        // Calculate pulse duration for each ESC
        targetSpeed[0] = instruction[YAW] + roll_pid + pitch_pid ;//- yaw_pid;
        targetSpeed[3] = instruction[THROTTLE] - roll_pid + pitch_pid ;//+ yaw_pid;
        targetSpeed[2] = instruction[ROLL] + roll_pid - pitch_pid ;//+ yaw_pid;
        targetSpeed[1] = instruction[PITCH] - roll_pid - pitch_pid ;//- yaw_pid;
   
   
    
}
void getFlightInstruction(int *actSpeed) {
    instruction[YAW]      = map(actSpeed[0], 0, 255, -180, 180);
    instruction[PITCH]    = map(actSpeed[1], 0, 255, 33, -33);
    instruction[ROLL]     = map(actSpeed[2], 0, 255, -33, 33);
    instruction[THROTTLE] = map(actSpeed[3], 0, 255, 0, 255); // Get some room to keep control at full speed
}


//-----------ACCELEROMETER-------------

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  accX = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  accY = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  accZ = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyroX = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyroY = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyroZ = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}

void calcAngle(){
   read_mpu_6050_data(); 
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

 
  }

//-----------------------------------------------------------------------------

void setup()
{
  // Debug console
  mySerial.begin(9600);

  Serial.begin(9600);  

  //---------------ACCELEROMETER-------------

  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  
  //---------------PID--------------------------
  
  
  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
  } 

  pinMode(FL_MOTOR, OUTPUT);
  pinMode(FR_MOTOR, OUTPUT);
  pinMode(BR_MOTOR, OUTPUT);
  pinMode(BL_MOTOR, OUTPUT);
  setup_mpu_6050_registers();
  
  read_mpu_6050_data(); 

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  
}

void loop()
{
//----------------------Speed----------------------------------//
  if(mySerial.available()>0)
{
  
  data="";
    delay(100);
    char c=mySerial.read();
    data+=c;
  if(data=="f"){ 
  actSpeed[0]-=1;
  actSpeed[1]-=1;
  actSpeed[2]+=1;
  actSpeed[3]+=1;
  }
else if(data == "o"){

  actSpeed[0]+=1;
  actSpeed[1]+=1;
  actSpeed[2]+=1;
  actSpeed[3]+=1;
}

if(data=="b"){ 
  actSpeed[0]+=1;
  actSpeed[1]+=1;
  actSpeed[2]-=1;
  actSpeed[3]-=1;
  }
else if(data == "m"){

actSpeed[0]-=1;
  actSpeed[1]-=1;
  actSpeed[2]-=1;
  actSpeed[3]-=1;
}

if(data=="l"){ 
actSpeed[0]-=1;
  actSpeed[1]+=1;
  actSpeed[2]-=1;
  actSpeed[3]+=1;
}
else if(data == "p"){
  int mean =(actSpeed[0]+actSpeed[1]+actSpeed[2]+actSpeed[3])/4;
actSpeed[0]=mean;
  actSpeed[1]=mean;
  actSpeed[2]=mean;
  actSpeed[3]=mean;
}

if(data=="r"){ 
actSpeed[0]+=1;
  actSpeed[1]-=1;
  actSpeed[2]+=1;
  actSpeed[3]-=1;
}
else if(data == "n"){
actSpeed[0]-=1;
  actSpeed[1]+=1;
  actSpeed[2]+=1;
  actSpeed[3]-=1;
}
for(int i=0;i<=3;i++){
  if(actSpeed[i]>255){
    actSpeed[i]=255;
    }
  if(actSpeed[i]<0){
    actSpeed[i]=0;
    }
  }

  
//-----------------ACCELEROMETER--------------
calcAngle();

//----------------------------------------------
   getFlightInstruction(actSpeed);

   
   calculateErrors();
//---------------------PID----------------------

 //if (isStarted()) {
        // 5. Calculate motors speed with PID controller
       
    //}
//----------------------------------------------
  
   pid(actSpeed);
     for(int i=0;i<=3;i++){
  if(targetSpeed[i]>255){
    targetSpeed[i]=255;
    }
  if(targetSpeed[i]<0){
    targetSpeed[i]=0;
    }
  }
       
  runIndividual(targetSpeed);
  Serial.print(targetSpeed[0]);
  Serial.print("  ");
  Serial.print(targetSpeed[1]);
  Serial.print("  ");
  Serial.print(targetSpeed[2]);
  Serial.print("  ");
  Serial.println(targetSpeed[3]);
 
 
  
 
}
}






