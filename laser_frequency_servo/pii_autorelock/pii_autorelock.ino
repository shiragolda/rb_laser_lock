/*
  PI control for single laser lock to photodiode signal
  Scanning mode - outputs ramp to piezo
  Locking mode - outputs PI correction to piezo (channel 0) + P correction to current (channel 1)
 */

#include <analogShield.h>
#include <SPI.h>  //required for ChipKIT but does not affect Arduino

typedef struct {
  String param_name;
  float param_value;
} Parameter ;

/* Select input pin (0 to 4) */
int PD_channel = 1;    // input pin for input signal

/* Measured quantities */
Parameter measured_sig_amp = {"Desired lock-point amplitude [mV]",350}; //the amplitude of the discriminator slope
Parameter loop_speed = {"Running loop speed (locking mode) [kHz]",12.8};

/* Input Initial PID Control ParameteParameterrs */
Parameter pterm_piezo = {"P (piezo)",0.01};   // proportional gain term on piezo
Parameter pterm_current = {"P (current)",2};   // proportional gain term on current
Parameter itime = {"Integration time constant [us]",500};    // integration time constant in microseconds
Parameter stime = {"Second integration time constant [us]",1000}; // second integration time (i squared) in microseconds
Parameter dtime = {"Derivative time constant [us]",2}; // derivative time constant in microseconds
Parameter fterm = {"Feedforward to current",0.1}; //feedforward scaling term
Parameter alpha = {"Low-pass filter constant alpha",0.9}; //proportional gain low-pass filter constant

/* Adjust Initial Ramp Parameters */
Parameter freq = {"Scan frequency [Hz]",98.7}; //in Hz
Parameter amp = {"Scan amplitude [V]", 1.5}; // in V

/* Global variables */
float set_point;
float lock_point;
float lock_point_time;
float lock_point_offset;
float PI_out = 0;
float PIs_out = 0;
float PIID_out = 0;
float P_out = 0;
float error;
float error_previous = 0;
float d_error;
float d_error_previous = 0;
double period = 1000000/freq.param_value; //in micros
unsigned long current_time;
unsigned long ramp_time;
long ramp_reset_time = 0;
byte byte_read;
int lock_state = 0;  // 0 is scanning, 1 is locked
int trigger_auto_relock = 0;
float accumulator = 0;
float accumulator_squared;
unsigned long loop_counter = 0;

float zerov = 32768.0; //Zero volts

void setup()  
{
 /* Open serial communications, initialize output ports: */
 SPI.setClockDivider(SPI_CLOCK_DIV2); //required for maximum loop speed

 Serial.begin(115200);
 Serial.println("Set line-ending to 'no line-ending'");
 Serial.println("");
 Serial.println("To toggle between scan/lock, type [y]."); 
 Serial.println("");
 Serial.println("Scan amplitude: " + String(amp.param_value,3) + " - to change, type [a]."); 
 Serial.println("Proportional gain on piezo channel: " + String(pterm_piezo.param_value, 4) + " - to change, type [p].");
 Serial.println("Proportional gain on current channel: " + String(pterm_current.param_value, 4) + " - to change, type [c].");
 Serial.println("Integral gain time: " + String(itime.param_value, 2) + " us - to change, type [i].");
 Serial.println("Integral-squared gain time: " + String(stime.param_value, 2) + " us - to change, type [s].");
 Serial.println("Feedforward scaling: " + String(fterm.param_value, 6) + " - to change, type [f].");
 Serial.println("Proportional gain low-pass filter constant: " + String(alpha.param_value, 6) + " - to change, type [l].");
  Serial.println("**Desired signal amplitude (approx) set to: " + String(measured_sig_amp.param_value,3) + " mV - to change, type [m].**");
 Serial.println("");
 Serial.println("scanning mode");
 Serial.println("");

 analog.write(zerov,zerov,zerov,zerov,true);
}

float ToVoltage(float bits) {
  return (bits-32768)/6553.6;
}

float ToBits(float voltage) {
  return voltage*6553.6+32768;
}

/*Generate ramp voltage */
float RampOut(long ramp_time) {
  float ramp;
  float offset = 2.5-amp.param_value/2; // centre the scan between 0 and 5 V
  if(ramp_time<=(period/2))
    ramp = (amp.param_value/(period/2))*ramp_time + offset;
  else 
    ramp = -(amp.param_value/(period/2))*ramp_time + 2*amp.param_value + offset;
  return ramp;
  }

/* Find the lockpoint - first minimum followed by a maximum with a difference close to the user-input signal amplitude*/
void GetSetPoint(void) {
  float max_val = -5;
  float min_val = 5;
  float max_val_time;
  float min_val_time;
  float slope;
  float sig_amp;
  int lock_point_found = 0;
  float lock_point_slope;

  sig_amp = measured_sig_amp.param_value/1000; // in V
  min_val = 5;
  max_val = -5;

  /* Start scan */
  ramp_reset_time = micros();
  do {
    current_time = micros(); //time in mircoseconds
    ramp_time = current_time-ramp_reset_time;
    analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
    float sig_read = ToVoltage(analog.read(PD_channel));

    /* Find the minimum */
    if(sig_read<min_val && ramp_time<period/2) {
      min_val = sig_read;
      min_val_time = ramp_time; 
      max_val = -5;
    }
    /* Find the maximum that follows the minimum - if the difference between the two is close to the 
       signal amplitude, the correct peak has been found */
    if(ramp_time>min_val_time && sig_read>max_val && ramp_time<period/2) {
      max_val = sig_read;
      max_val_time = ramp_time;
      slope = max_val - min_val;
      if(abs(slope-sig_amp)<0.25*sig_amp) {
        lock_point = (max_val+min_val)/2.0;
        lock_point_time = (max_val_time + min_val_time)/2.0;
        lock_point_offset = RampOut(lock_point_time);
        lock_point_found = 1;
        lock_point_slope = slope;
      }
    }      
  
  } while(ramp_time<period);
    
  Serial.print("Lock-point signal amplitude: ");
  Serial.print(lock_point_slope*1000,2);
  Serial.print(" mV; Lock-point time (from start of scan): ");
  Serial.print(lock_point_time/1000.);
  Serial.println(" ms");
  
  if(lock_point_found==0) {
    accumulator = 100; //triggers relocking
    Serial.println("Lock-point not found");
  }
  if(lock_point_found==1);
    Serial.println("Relocked");
  
}

/* CalculatePID(): */
void GetPID(void) { 
  float act_signal = ToVoltage(analog.read(PD_channel)); 
  float dt = 1000/loop_speed.param_value; //Loop period (locking mode) in microseconds
  error = set_point-act_signal; 
  error = (alpha.param_value*error_previous) + (1-alpha.param_value)*error;
  
  P_out = pterm_piezo.param_value*(error);
  
  //Limit output voltage to -4V < Vout < 4V */
  if(P_out>=4)
    P_out = 4;
  if(P_out<=-4)
    P_out = -4;
    
  accumulator += error;  // accumulator is sum of errors (for integral gain term)
  accumulator_squared += error + (1/stime.param_value)*accumulator*dt;

  d_error = error-error_previous;
  d_error = 0.9*d_error_previous + 0.1*d_error;
  
  //PI_out = P_out+(pterm_piezo.param_value*(1/itime.param_value)*accumulator*dt);
  //PIs_out = P_out+(pterm_piezo.param_value*(1/itime.param_value)*accumulator_squared*dt);
  PIID_out = P_out+((pterm_piezo.param_value*(1/itime.param_value)*accumulator_squared*dt)+(dtime.param_value/dt)*(d_error));
  
  //Limit output voltage */
  float limit = (5-amp.param_value)/2;
  if(PIID_out>=limit)
    PIID_out = limit;
  if(PIID_out<=-limit)
    PIID_out = -limit;

  error_previous = error;
  d_error_previous = d_error;
}

/* Accept a serial input float */
float floatIn() {
  while(!Serial.available()){} //Wait for serial input
  return Serial.parseFloat(); //parse the next float
}

Parameter UpdateParameter(Parameter param) {
  Serial.print(param.param_name + " = ");
  Serial.print(param.param_value,6);
  Serial.println(" - enter a new value for " + param.param_name + ":");
  float new_value = floatIn();
  Serial.print(param.param_name + " = ");
  Serial.println(new_value,6);
  Parameter new_param = {param.param_name,new_value};
  return new_param;
}



void loop() // run over and over
{
  byte_read = Serial.read();
  
  /* Listen for parameter adjust commands */
  if(byte_read == 'a') {amp = UpdateParameter(amp); }
  if(byte_read == 'r') {freq = UpdateParameter(freq); }
  if(byte_read == 'f') {fterm = UpdateParameter(fterm); }
  if(byte_read == 'l') {alpha = UpdateParameter(alpha); }
  if(byte_read == 'p') {pterm_piezo = UpdateParameter(pterm_piezo); }
  if(byte_read == 'c') {pterm_current = UpdateParameter(pterm_current); }
  if(byte_read == 'i') {itime = UpdateParameter(itime); }
  if(byte_read == 's') {stime = UpdateParameter(stime); }
  if(byte_read == 'd') {dtime = UpdateParameter(dtime); }
  if(byte_read == 'm') {measured_sig_amp = UpdateParameter(measured_sig_amp); }
  if(byte_read == 'o') {loop_speed = UpdateParameter(loop_speed); }
  
  period = 1000000/freq.param_value; //in micros

 
  /* Listen for a scan/lock toggle command: */
  if(byte_read == 'y') {
    lock_state = !lock_state; //toggle the lock state
    if(lock_state == 0) {
      Serial.println("scanning mode");
    }
    if(lock_state == 1) {
      Serial.println("locking mode");
      loop_counter = 0; //reset a bunch of parameters
      accumulator = 0;
      accumulator_squared = 0;
      error_previous = 0;
      d_error_previous = 0;
      GetSetPoint(); //find the setpoint
      set_point = lock_point;
      
      // Scan once more until lock_point_time so system is in the capture range:
      ramp_reset_time = micros();
      do {
        current_time = micros(); //time in microseconds
        ramp_time = current_time-ramp_reset_time;
        analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
      } while(ramp_time<lock_point_time);

    }
  }

  
  /* Write output for scanning mode: */
  if(lock_state == 0) {
    ramp_reset_time = micros(); 
    do {
      current_time = micros(); //time in mircoseconds
      ramp_time = current_time-ramp_reset_time;
      analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
    } while(ramp_time<period);  
  }
  
  /* Write output for locking mode: 
  To get the sign right - send the ramp to the piezo and P_out to the current. 
  Choose a large proportional gain. When it is applied, the slope of the 
  absorption feature should become noticeably less steep.  */
  if(lock_state == 1) {
    loop_counter ++;
    GetPID();  //OUT0 to the piezo, OUT1 to the current controller
    analog.write(ToBits(PIID_out+lock_point_offset),ToBits(-1.0*(pterm_current.param_value/pterm_piezo.param_value)*P_out-1.0*fterm.param_value*PIID_out),true);

  /* AUTO-RELOCK */
    if(abs(accumulator)>50) {
      Serial.println("Out of lock!");
      loop_counter = 0;
      accumulator = 0;
      accumulator_squared = 0;
      GetSetPoint();
      set_point = lock_point;
      // Scan once more until lock_point_time so system is in the capture range:
      ramp_reset_time = micros();
      do {
        current_time = micros(); //time in mircoseconds
        ramp_time = current_time-ramp_reset_time;
        analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
      } while(ramp_time<lock_point_time);
    }
    
    /*Write to serial for data logging in python */
    if(loop_counter%75000 == 0) {
      Serial.print(error,4);
      Serial.print(',');
      float correction = PIID_out+lock_point_offset;
      Serial.println(correction,4);
    } 
  }
 

}
