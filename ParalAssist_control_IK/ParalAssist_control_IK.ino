#include <Wire.h>
#include <PWM.h>
#include <Servo.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
//#define Wire.write(x) Wire.send(x)
//#define Wire.read() Wire.receive()
#endif


static uint8_t nunchuck_buf[6];   // array to store nunchuck data,

// Uses port C (analog in) pins as power & ground for Nunchuck
static void nunchuck_setpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
  DDRC |= _BV(pwrpin) | _BV(gndpin);
  PORTC &= ~ _BV(gndpin);
  PORTC |=  _BV(pwrpin);
  delay(100);  // wait for things to stabilize
}

// initialize the I2C system, join the I2C bus,
// and tell the nunchuck we're talking to it
static void nunchuck_init(){
  Wire.begin();                // join i2c bus as master
  Wire.beginTransmission(0x52);// transmit to device 0x52
  #if (ARDUINO >= 100)
    Wire.write((uint8_t)0x40);// sends memory address
    Wire.write((uint8_t)0x00);// sends sent a zero.
  #else
    Wire.send((uint8_t)0x40);// sends memory address
    Wire.send((uint8_t)0x00);// sends sent a zero.
  #endif
  Wire.endTransmission();// stop transmitting
}

// Send a request for data to the nunchuck
// was "send_zero()"
static void nunchuck_send_request(){
  Wire.beginTransmission(0x52);// transmit to device 0x52
  #if (ARDUINO >= 100)
    Wire.write((uint8_t)0x00);// sends one byte
  #else
    Wire.send((uint8_t)0x00);// sends one byte
  #endif
  Wire.endTransmission();// stop transmitting
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
static char nunchuk_decode_byte (char x){
  x = (x ^ 0x17) + 0x17;
  return x;
}

// Receive data back from the nunchuck,
// returns 1 on successful read. returns 0 on failure
static int nunchuck_get_data(){
  int cnt = 0;
  Wire.requestFrom (0x52, 6);// request data from nunchuck
  while (Wire.available ()) {
    // receive byte as an integer
    #if (ARDUINO >= 100)
      nunchuck_buf[cnt] = nunchuk_decode_byte( Wire.read() );
    #else
      nunchuck_buf[cnt] = nunchuk_decode_byte( Wire.receive() );
    #endif
    cnt++;
  }
    nunchuck_send_request();  // send request for next data payload
    // If we recieved the 6 bytes, then go print them
    if (cnt >= 5) {
      return 1;   // success
    }
  return 0; //failure
}

// Print the input data we have recieved
// accel data is 10 bits long
// so we read 8 bits, then we have to add
// on the last 2 bits.  That is why I
// multiply them by 2 * 2
static void nunchuck_print_data(){
  static int i = 0;
  int joy_x_axis = nunchuck_buf[0];
  int joy_y_axis = nunchuck_buf[1];
  int accel_x_axis = nunchuck_buf[2]; // * 2 * 2;
  int accel_y_axis = nunchuck_buf[3]; // * 2 * 2;
  int accel_z_axis = nunchuck_buf[4]; // * 2 * 2;
  int z_button = 0; 
  int c_button = 0;

  // byte nunchuck_buf[5] contains bits for z and c buttons
  // it also contains the least significant bits for the accelerometer data
  // so we have to check each bit of byte outbuf[5]
  if ((nunchuck_buf[5] >> 0) & 1)
    z_button = 1;
  if ((nunchuck_buf[5] >> 1) & 1)
    c_button = 1;

  if ((nunchuck_buf[5] >> 2) & 1)
    accel_x_axis += 1;
  if ((nunchuck_buf[5] >> 3) & 1)
    accel_x_axis += 2;

  if ((nunchuck_buf[5] >> 4) & 1)
    accel_y_axis += 1;
  if ((nunchuck_buf[5] >> 5) & 1)
    accel_y_axis += 2;

  if ((nunchuck_buf[5] >> 6) & 1)
    accel_z_axis += 1;
  if ((nunchuck_buf[5] >> 7) & 1)
    accel_z_axis += 2;

  Serial.print(i, DEC);
  Serial.print("\t");

  Serial.print("joy:");
  Serial.print(joy_x_axis, DEC);
  Serial.print(",");
  Serial.print(joy_y_axis, DEC);
  Serial.print("  \t");

  Serial.print("acc:");
  Serial.print(accel_x_axis, DEC);
  Serial.print(",");
  Serial.print(accel_y_axis, DEC);
  Serial.print(",");
  Serial.print(accel_z_axis, DEC);
  Serial.print("\t");

  Serial.print("but:");
  Serial.print(z_button, DEC);
  Serial.print(",");
  Serial.print(c_button, DEC);

  Serial.print("\r\n");  // newline
  i++;
}

// returns zbutton state: 1=pressed, 0=notpressed
static int nunchuck_zbutton()
{
  return ((nunchuck_buf[5] >> 0) & 1) ? 0 : 1;  // voodoo
}

// returns cbutton state: 1=pressed, 0=notpressed
static int nunchuck_cbutton()
{
  return ((nunchuck_buf[5] >> 1) & 1) ? 0 : 1;  // voodoo
}

// returns value of x-axis joystick
static int nunchuck_joyx()
{
  return nunchuck_buf[0];
}

// returns value of y-axis joystick
static int nunchuck_joyy()
{
  return nunchuck_buf[1];
}


/* acceleration is not used in the this application
// returns value of x-axis accelerometer
static int nunchuck_accelx(){
  return nunchuck_buf[2];   // FIXME: this leaves out 2-bits of the data
}

// returns value of y-axis accelerometer
static int nunchuck_accely(){
  return nunchuck_buf[3];   // FIXME: this leaves out 2-bits of the data
}

// returns value of z-axis accelerometer
static int nunchuck_accelz(){
  return nunchuck_buf[4];   // FIXME: this leaves out 2-bits of the data
}
*/


///////////// Global Variables Declaration /////////////////
const int state_change_threshold = 5;
const int homing_threshold = 30;
const int joystick_disp_threshold_l = 50;
const int joystick_disp_threshold_h = 200;

// these motor_angle_* variables give software joint limit in each joint. In order to have consistent joint limits 
// the limit values used in comparison with motor_pwm_* should be function of m*_speed and these motor*_angle_* variables
const int motor1_angle_low = 42;
const int motor2_angle_low = 32;
const int motor3_angle_low = 32;
const int motor4_angle_low = 8; 
const int motor1_angle_high = 165;
const int motor2_angle_high = 165;
const int motor3_angle_high = 165;
const int motor4_angle_high = 31; 

const int m1_speed = 2;
const int m2_speed = 2;
const int m3_speed = 2;
const int m4_speed = 4;

int loop_cnt = 0;
byte joyx, joyy, zbut, cbut;
byte accx, accy, accz; // we dont use accelerations but just keeping it
bool state_z = false; // z=0 ==> M1;    z=1 ==> M4;
bool state_c = false; // c=0 ==> M2;    c=1 ==> M3;
bool z_changed = false; // indicate whether it recorded state change before homing
bool c_changed = false; // indicate whether it recorded state change before homing
int z_counter = 0; // counter for the z button. accounts for debouncing
int c_counter = 0; // counter for the c button. accounts for debouncing

int motor_pwm_1 = 100; // set as 0 degree for now
int motor_pwm_2 = 200; // 
int motor_pwm_3 = 120;
int motor_pwm_4 = 7; // 81 for mid point

int gripperX; // X coordinate of gripper
int gripperY; // Y coordinate of gripper
int gripperZ; // Z coordinate of gripper

int gripperX_low = 0; // X coordinate min
int gripperY_low = 0; // Y coordinate min
int gripperZ_low = 0; // Z coordinate min

int gripperX_high = 0; // X coordinate max
int gripperY_high = 0; // Y coordinate max
int gripperZ_high = 0; // Z coordinate max

// ****** Hardware constants **********//
const float a1 = 30; // length between {1} and {2} projected by x1 unit vector
const float a2 = 30; // length of first link
const float a3 = 30; // length of second link
const float a4 = 30; // length of thirdlink + gripper
const float d2 = 30; // legnth betwen {1} and {2} projected by z1 unit vector

const float theta1_l = -135/2;
const float theta1_u = theta1_l + 135/2;
const float theta2_l = -20;
const float theta2_u = theta2_l + 135;
const float theta3_l = -20; 
const float theta3_u = theta3_l + 135;

const float initial_theta1 = 0; // used at the start of the program and for destiniation of homing funciton
const float initial_theta2 = 0; // used at the start of the program and for destiniation of homing funciton
const float initial_theta3 = 0; // used at the start of the program and for destiniation of homing funciton

float thetas[2][3] = { // solutions what wil be updated by IK function. each row vector correspond to one set of solution
  {initial_theta1, initial_theta2, initial_theta3},
  {initial_theta1, initial_theta2, initial_theta3}
};

bool solution_validity[2] = {true, true};

bool plotter = true;

Servo gripper;
/////////////////////////////////////////////////////////////

void _print(int mappedx, int mappedy, bool plotter) { // used to print the status during debugging process
  if (plotter == false){
    Serial.print("\nZ Button:  ");
    Serial.print(zbut);
    Serial.print(" | C Button:  ");
    Serial.print(cbut);
    Serial.print(" | mappedx:  ");
    Serial.print(mappedx);
    Serial.print(" | mappedy:  ");
    Serial.print(mappedy);
    Serial.print(" | state_z:  ");
    Serial.print(state_z);
    Serial.print(" | state_c:  ");
    Serial.print(state_c);
    Serial.print(" | z_counter: ");
    Serial.print(z_counter);
    Serial.print(" | c_counter: ");
    Serial.print(c_counter);
    Serial.print("\n");
    
      // print x value
    if (mappedx < 50){
      Serial.print("left | ");
    }
    else if (mappedx > 200){
      Serial.print("right | ");
    }
    else{
      Serial.print("dead | ");
    }
    if (state_z == false){
      Serial.print("motor_pwm_1:");
      Serial.print(" ");
      Serial.println(motor_pwm_1);
    }
    else{
      Serial.print("motor_pwm_4:");
      Serial.print(" ");
      Serial.println(motor_pwm_4);
    }

    // print y value
    if (mappedy < 50){
      Serial.print("pull | ");
    }
    else if (mappedy > 200){
      Serial.print("push | ");
    }
    else{
      Serial.print("dead | ");
    }
    if (state_c == 0){
      Serial.print("motor_pwm_2:");
      Serial.print(" ");
      Serial.println(motor_pwm_2);
      Serial.print(" ");
    }
    else{
      Serial.print("motor_pwm_3:");
      Serial.print(" ");
      Serial.println(motor_pwm_3);
      Serial.print(" ");
    }

    Serial.print("\n");
  }
  else{
    // print x value
    Serial.print("z_counter:"); Serial.print(z_counter); Serial.print(", ");
    Serial.print("c_counter:"); Serial.print(c_counter); Serial.print(", ");
    // Serial.print("mappedx:"); Serial.print(mappedx); Serial.print(", ");
    // Serial.print("mappedy:"); Serial.print(mappedy); Serial.print(", ");
    Serial.print("motor_pwm_1:"); Serial.print(motor_pwm_1); Serial.print(", ");
    Serial.print("motor_pwm_2:"); Serial.print(motor_pwm_2); Serial.print(", ");
    Serial.print("motor_pwm_3:"); Serial.print(motor_pwm_3); Serial.print(", ");
    Serial.print("motor_pwm_4:"); Serial.print(motor_pwm_4); Serial.print(", ");
    Serial.println();
  }
}


#define motorPIN1 3 // elbow joint
#define motorPIN2 9 
#define motorPIN3 10
#define motorPIN4 5 // gripper


bool set_pin_frequency(){ // set PWM frequencies in the analog output pins
  bool pin1set = SetPinFrequencySafe(motorPIN1,300);  
  bool pin2set = SetPinFrequencySafe(motorPIN2,300);
  bool pin3set = SetPinFrequencySafe(motorPIN3,300);
  bool pin4set = SetPinFrequency(motorPIN4, 50);

  if(pin1set){
    pinMode(motorPIN1,OUTPUT); // pin 3
    pwmWrite(motorPIN1, motor_pwm_1);
  }
  else {
    Serial.print("pin1set: false\n");
  }

  if(pin2set){
    pinMode(motorPIN2,OUTPUT); // pin 9
    pwmWrite(motorPIN2, motor_pwm_2);
  }
  else {
    Serial.print("pin2set: false\n");
  }

  if(pin3set){
    pinMode(motorPIN3,OUTPUT); // pin 10
    pwmWrite(motorPIN3, motor_pwm_3);
  }
  else {
    Serial.print("pin3set: false\n");
  }

  if(pin4set){
    pinMode(motorPIN4,OUTPUT); // pin 5
    pwmWrite(motorPIN4, motor_pwm_4);
  }
  else {
    Serial.print("pin4set: false\n");
  }

  if (pin1set && pin2set && pin3set && pin4set) {
    return true;
  }
  else{
    return false;
  }
}

float rad2deg(float radians){
  float degrees = (radians * 4068) / 71;
  return degrees;
}

float sqr(float x){
  return (x * x);
}

int deg2pwm(float theta, int joint_number){ // converts angle to pwm value used for motor control. use for joint 1,2,3
  int pwm_value = 0;
  if (joint_number == 1){
    pwm_value = map(theta, theta1_l, theta1_u, 0, 255); 
  }
  else if (joint_number == 2){
    pwm_value = map(theta, theta2_l, theta2_u, 0, 255); 
  }
  else if (joint_number == 3){
    pwm_value = map(theta, theta3_l, theta3_u, 0, 255); 
  }
  // else if (joint_number == 4){ // do we really need this..?
  //   pwm_value = theta;
  // }
  else{ // we shouldnt go here, maybe call exception?
    return 0;
  }
  return pwm_value;
}

float pwm2deg(int pwm, int joint_number){
  float theta_value = 0;
  if (joint_number == 1){
    theta_value = map(pwm, 0, 255, theta1_l, theta1_u); 
  }
  else if (joint_number == 2){
    theta_value = map(pwm, 0, 255, theta2_l, theta2_u); 
  }
  else if (joint_number == 3){
    theta_value = map(pwm, 0, 255, theta3_l, theta3_u); 
  }
  // else if (joint_number == 4){ // do we really need this..?
  //   pwm_value = theta;
  // }
  else{ // we shouldnt go here, maybe call exception?
    return 0;
  }
  return theta_value;
}

void InKin(int Xd_int, int Yd_int, int Zd_int, float (& thetas)[2][3], bool (& solution_validity) [2] ){ // calculates set of solution that will locate ghe grippper to Xd, Yd, and Zd coordinate
  // the solution will be stored at the float vector passed by reference.
  // also it will set flags to "solution_validity". if flag is false, then the solution is unachievable by the ParalAssist

  // first make the input as floats
  float Xd = static_cast< float >(Xd_int);
  float Yd = static_cast< float >(Yd_int);
  float Zd = static_cast< float >(Zd_int);

  // update the solution_validity to default
  solution_validity[0] = true;
  solution_validity[1] = true;

  // check if there can be valid solution
  float r1 = sqrt(Xd*Xd + Yd*Yd); // r1 = sqrt(Xd^2 + Yd^2)
  if (r1 > (a1 + sqrt(sqr(a2)+sqr(a3))) ){ // point cannot be reached 
    solution_validity[0] = false;
    solution_validity[1] = false;
  }
  else{ // proceed calculations
    float theta1 = rad2deg(atan2(Yd, Xd)); // atan2() returns radian value, we need degrees
    if ((theta1 < theta1_l) || (theta1 > theta1_u)){ // outside the joint limit1
      solution_validity[0] = false;
      solution_validity[1] = false;
    }
    else{
      float r2_sq = sqr(r1-a1) + sqr(Zd-d2);
      float cos3 = (r2_sq - (sqr(a3) + sqr(a4)))/(2*a3*a4);
      float sin3 = sqrt(1 - sqr(cos3));
      float theta3_1 = rad2deg(atan2(sin3, cos3)); // _1 is part of first solution set
      float theta3_2 = rad2deg(atan2(-1*sin3, cos3)); // _2 is part of second solution set
      if ((theta3_1 < theta3_l) || (theta3_1 > theta3_u)){
        solution_validity[0] = false;
      }
      if ((theta3_2 < theta3_l) || (theta3_2 > theta3_u)){
        solution_validity[1] = false;
      }
      float alpha = rad2deg(atan2(Zd-d2, r1-a1));
      float cos_beta = (r2_sq + sqr(a2) - sqr(a3))/(2*sqrt(r2_sq)*a2);   
      float sin_beta = sqrt(1+sqr(cos_beta));
      float theta2_1 = alpha + rad2deg(atan2(sin_beta, cos_beta));
      float theta2_2 = alpha + rad2deg(atan2(-1*sin_beta, cos_beta));
      if ((theta2_1 < theta2_l) || (theta2_1 > theta2_u)){
        solution_validity[0] = false;
      }
      if ((theta2_2 < theta2_l) || (theta2_2 > theta2_u)){
        solution_validity[1] = false;
      }

      // manipulate the reference
      thetas[0][0] = theta1;
      thetas[0][1] = theta2_1;
      thetas[0][2] = theta3_1;
      thetas[1][0] = theta1;
      thetas[1][1] = theta2_2;
      thetas[1][2] = theta3_2;  
    }
  }
}

int findOptSol(float (& thetas)[2][3], int pwm1, int pwm2, int pwm3, bool * solution_validity ){ // find which solution set from thetas require less movement, hence optimal. returns ideal solution set number
  if ((solution_validity[0] || solution_validity[1]) == false){ // both flags are false, no possible solution
    return 0;
  }
  else if ((solution_validity[0]==true) && (solution_validity[1]==false)){ // only sol1 is valid
    return 1;
  }
  else if ((solution_validity[0]==false) && (solution_validity[1]==true)){ // only sol2 is valid
    return 2;
  }
  else{ // both solutions are valid, we need to compare to find the optimum solution
    float angle1 = pwm2deg(pwm1, 1);
    float angle2 = pwm2deg(pwm2, 2);
    float angle3 = pwm2deg(pwm3, 3);
    float dist1 = 0;
    float dist2 = 0;
    float w1 = 35; // weight for first motor
    float w2 = 60; // weight for second motor
    float w3 = 35; // weight for third motor

    dist1 = sqrt(w1*sqr(angle1-thetas[0][0]) + w2*sqr(angle2-thetas[0][1]) + w3*sqr(angle3-thetas[0][2]));
    dist2 = sqrt(w1*sqr(angle1-thetas[1][0]) + w2*sqr(angle2-thetas[1][1]) + w3*sqr(angle3-thetas[1][2]));
    if (dist1 <= dist2){
      return 1; // sol1 is optimum
    }
    else{
      return 2; // sol2 is optimum
    }
  }
  return 0; // we should not reach this point.
}


void setup() {
  Serial.begin(9600);
  nunchuck_setpowerpins();
  nunchuck_init(); // send the initilization handshake
  if(plotter == false){
    // Serial.print("Wii Nunchuck Ready\n");
  }
  else{
    Serial.println("mappedx mappedy motor_pwm_1 motor_pwm_2 motor_pwm_3 motor_pwm_4");
  }

  InitTimers();                    // won't touch timer0

  bool pins_are_set;
  do{
    pins_are_set = set_pin_frequency();
  } while (pins_are_set != true);

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //Move M1,2,3 to preset position and update xyz coordinate. TBD
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
}

void loop() {
  if ( loop_cnt > 10 ) { // every 1000 msecs get new data
    loop_cnt = 0;

    nunchuck_get_data(); // read values from the nunchuck

    // check C and Z status
    zbut = nunchuck_zbutton();  //  0 - 1
    cbut = nunchuck_cbutton();  //  0 - 1

    // check Z button 
    // Z button now is only used for homing
    if (zbut){
      z_counter++; // counter resets when either state changes or HOMING is called
      if (z_counter > homing_threshold){
        if (plotter== false){
          Serial.print("------------- homing ---------------\n");
        }
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //Move M1,2,3 to preset position and update xyz coordinate. TBD
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        z_counter = 0;
      }
    }
    else{ // bouncing button
      z_counter = 0; // reset counter 
    }

    // check C button 
    // C button false -> x and y of joy controls x and y of end factor
    // C button true -> x and y of joy controls gripper open,close and z of end factor
    if (cbut){
      c_counter++; // counter resets when either state changes or  is called
      if (c_counter > state_change_threshold){
        if (plotter == false){
          Serial.print("------------- toggle c_state ---------------\n");
        }
        state_c = !state_c;
        c_counter = 0;
      }
    }
    else{
      c_counter = 0; // reset counter 
    }

    // check joystick value and control motor
    joyx = nunchuck_joyx();     //  15 - 221
    joyy = nunchuck_joyy();     //  29 - 229
    int mappedx = map(joyx, 15, 221, 0, 255);
    int mappedy = map(joyy, 15, 221, 0, 255);
    // int motor4angle = map(motor_pwm_4, 32, 195, 0, 180);
    
    _print(mappedx, mappedy, plotter); // third parameter controls the monitor(false)/plotter(true) 
    
    // deadzone checking and motor actuation
    {
      if (mappedx < joystick_disp_threshold_l){ // we consider joystic's x value first
        if (state_c == false){ // control motor1
          if (gripperX > gripperX_low){
            gripperX -= gripperX;
            //pwmWrite(motorPIN1, motor_pwm_1);
          }
        }
        else{ // control motor4
          if (motor_pwm_4 > motor4_angle_low){
            motor_pwm_4 -= m4_speed;
            // motor4angle = map(motor_pwm_4, 32, 195, 0, 180);
            pwmWrite(motorPIN4, motor_pwm_4);
          }
        }
      }
      else if (mappedx > joystick_disp_threshold_h){
        if (state_c == false){ // control motor2
          if (gripperX < gripperX_high){
            gripperX += gripperX;
            //pwmWrite(motorPIN1, motor_pwm_1);
          }
        }
        else{
          if (motor_pwm_4 < motor4_angle_high){
            motor_pwm_4 += m4_speed;
            // motor4angle = map(motor_pwm_4, 32, 195, 0, 180);
            pwmWrite(motorPIN4, motor_pwm_4);
          }
        }
      }
      else{ // now we consider y values
        if (mappedy < joystick_disp_threshold_l){ // we consider joystic's x value first
          if (state_c == false){ // control motor2   
            if (gripperY > gripperY_low){
              gripperY -= gripperY;
              //pwmWrite(motorPIN2, motor_pwm_2);
            }
          }
          else{ // control motor3
            if (gripperZ > gripperZ_low){
              gripperZ -= gripperZ;
              //pwmWrite(motorPIN3, motor_pwm_3);
            }
          }
        }
        else if (mappedy > joystick_disp_threshold_l){
          if (state_c == false){ // control motor2
            if (gripperY < gripperY_high){
              gripperY += gripperY; 
              //pwmWrite(motorPIN2, motor_pwm_2);
            }
          }
          else{ // control motor3
            if (gripperZ < gripperZ_high) {
              gripperZ += gripperZ;
              //pwmWrite(motorPIN3, motor_pwm_3);
            }
          }
        }
      }
    }
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //send coordinates to IK function and use pwmWrite function to update angles.
    //TBD
  }
  loop_cnt++;
  // delay(1);

}
