// See http://blog.solutions-cubed.com/pid-motor-control-with-an-arduino/:
/*
  PI control for single laser lock to photodiode signal
  Scanning mode - outputs ramp to piezo
  Locking mode - outputs PI correction to piezo (channel 0) + P correction to current (channel 1)
 */

#include <analogShield.h>
#include <SPI.h>  //required for ChipKIT but does not affect Arduino

/* Select input pin (0 to 4) */
int PD_channel = 1;    // input pin for input signal

/* Input Initial PID Control Parameters */
float pterm_piezo = 0.03;   // proportional gain term on piezo
float pterm_current = 0.1;   // proportional gain term on piezo
float itime = 10000;    // integration time constant in microseconds
float fterm = 0.1; //feedforward scaling term
float alpha = 0.4; //proportional gain low-pass filter constant

/* Adjust Initial Ramp Parameters */
float freq = 10; //in Hz
float amp = 0.4; // in V

/* Global variables */
float set_point;
float lock_point;
float lock_point_time;
float lock_point_offset;
float PI_out = 0;
float P_out = 0;
float error;
float p_prime;
double period = 1000000/freq; //in micros
unsigned long current_time;
unsigned long ramp_time;
long ramp_reset_time = 0;
byte byte_read;
int lock_state = 0;  // 0 is scanning, 1 is locked
int trigger_auto_relock = 0;
float offset = 0;
float accumulator = 0;
unsigned long loop_counter = 0;

float zerov = 32768.0; //Zero volts

void setup()  
{
 /* Open serial communications, initialize output ports: */
 SPI.setClockDivider(SPI_CLOCK_DIV2); //required for maximum loop speed

 Serial.begin(115200);
 Serial.println("Set line-ending to Newline");
 Serial.println("To toggle between scan/lock, type [y]."); 
 Serial.println("Scan amplitude: " + String(amp,2) + " - to change, type [a]."); 
 Serial.println("Proportional gain on piezo channel: " + String(pterm_piezo, 2) + " - to change, type [p].");
 Serial.println("Proportional gain on current channel: " + String(pterm_current, 2) + " - to change, type [c].");
 Serial.println("Integral gain time: " + String(itime/1000, 2) + " ms - to change, type [i].");
 Serial.println("Feedforward scaling: " + String(fterm, 6) + " - to change, type [f].");
 Serial.println("Proportional gain low-pass filter constant: " + String(alpha, 6) + " - to change, type [l].");
 Serial.println("scanning mode");

 analog.write(zerov,zerov,zerov,zerov,true);
}

float ToVoltage(float bits) {
  return (bits-32768)/6553.6;
}

float ToBits(float voltage) {
  return voltage*6553.6+32768;
}

/* CalculatePID(): */
void GetPID(void) { 
  float act_signal = ToVoltage(analog.read(PD_channel)); 
  error = set_point-act_signal; 
  p_prime;
  P_out = pterm_piezo*(error);
  P_out = (alpha*p_prime) + (1-alpha)*P_out;
  //Limit output voltage to -5V < Vout < 5V */
  if(P_out>=5)
    P_out = 5;
  if(P_out<=-5)
    P_out = -5;
    
  p_prime = P_out;
  accumulator += error;  // accumulator is sum of errors (for integral gain term)
  
//  Serial.print(error,4);
//  Serial.print(",");
//  Serial.println(accumulator);
  
  float dt = 1000; //Approximate time in microseconds that the program takes between updates
  
  PI_out = P_out+(pterm_piezo*(1/itime)*accumulator*dt);
  //Limit output voltage to -5V < Vout < 5V */
  if(PI_out>=5)
    PI_out = 5;
  if(PI_out<=-5)
    PI_out = -5;
}

/*Generate ramp voltage */
float RampOut(long ramp_time) {
  float ramp;
  if(ramp_time<=(period/2))
    ramp = (amp/(period/2))*ramp_time;
  else 
    ramp = -(amp/(period/2))*ramp_time + 2*amp;
  return ramp;
  
  }

/* Scan and find the minimum. Scan and find the first maximum following the minimum.
   Calculate midpoint*/
void GetSetPoint(void) {
  float max_val = -5;
  float min_val = 5;
  float max_val_time;
  float min_val_time;
  ramp_reset_time = micros();
  do {
    current_time = micros(); //time in mircoseconds
    ramp_time = current_time-ramp_reset_time;
    analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
    float sig_read = ToVoltage(analog.read(PD_channel));
    if(sig_read<min_val && ramp_time<period/2) {
      min_val = sig_read;
      min_val_time = ramp_time; 
    }      
  } while(ramp_time<period);
  
  ramp_reset_time = micros();
  do {
    current_time = micros(); //time in mircoseconds
    ramp_time = current_time-ramp_reset_time;
    analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
    float sig_read = ToVoltage(analog.read(PD_channel));  
    if(ramp_time>min_val_time && sig_read>max_val && ramp_time<period/2) {
      max_val = sig_read;
      max_val_time = ramp_time;
    }
  } while(ramp_time<period);
  
  lock_point = (max_val+min_val)/2.0;
  lock_point_time = (max_val_time + min_val_time)/2.0;
  lock_point_offset = RampOut(lock_point_time);
}

/* Accept a serial input float */
float NumIn(void) {
  //Wait for an input:
    while(!Serial.available()){}
    
    //Throw away the newline printed after "k", where k is the keyletter
    if(Serial.available())
      byte Garbage = Serial.read();
      
    String StringIn = "";
    do {
      byte_read = Serial.read();
      //The following will ignore all the "pauses" read in:
      if(isDigit(byte_read) || byte_read == 46)
        StringIn += (char)byte_read;
    } while(byte_read!=10);
    
    return StringIn.toFloat();
}


void loop() // run over and over
{
  byte_read = Serial.read();

  if(byte_read == 'a') {
    Serial.println("Scan amplitude = " + String(amp) + " V - enter a new value for amplitude:");
    amp = NumIn();
    if(amp>5)
      amp = 5;
    Serial.println("amplitude set to " + String(amp,2) + " V");
  }

  if(byte_read == 'f') {
    Serial.println("Feedforward scale = " + String(fterm,6) + " - enter a new value:");
    fterm = NumIn();
    Serial.println("Feedforward scale set to " + String(fterm,6));
  }

  if(byte_read == 'l') {
    Serial.println("Alpha = " + String(alpha,3) + " - enter a new value:");
    alpha = NumIn();
    Serial.println("Alpha set to " + String(alpha,6));
  }
    
  /* Listen for a scan/lock toggle command: */
  if(byte_read == 'y') {
    lock_state = !lock_state;
    if(lock_state == 0) {
      offset = 0;
      Serial.println("scanning mode");
    }
    if(lock_state == 1) {
      Serial.println("locking mode");
      offset = 0;
      loop_counter = 0;
      accumulator = 0;
      GetSetPoint();
      set_point = lock_point;

      Serial.print("Lock-point: ");
      Serial.print(lock_point);
      Serial.print(" V, Lock-point time: ");
      Serial.print(lock_point_time/1000.);
      Serial.println(" ms");
      
      // Scan once more until lock_point_time so system is in the capture range:
      ramp_reset_time = micros();
      do {
        current_time = micros(); //time in mircoseconds
        ramp_time = current_time-ramp_reset_time;
        analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
      } while(ramp_time<lock_point_time);

    }
  }


  /* Listen for a p_piezo adjust command: */
  if(byte_read == 'p') {
    Serial.println("P (piezo) = " + String(pterm_piezo,2) + " - enter a new value for P:");
    pterm_piezo = NumIn();
    Serial.println("P (piezo) set to " + String(pterm_piezo,2));
  }
    
  /* Listen for a p_current adjust command: */
  if(byte_read == 'c') {
    Serial.println("P (current) = " + String(pterm_current,2) + " - enter a new value for P:");
    pterm_current = NumIn();
    Serial.println("P (current) set to " + String(pterm_current,2));
  }
    
  /* Listen for an i adjust command: */
  if(byte_read == 'i') {
    Serial.println("I = " + String(itime/1000,2) + " ms - enter a new value for I:");
    itime = NumIn()*1000;
    Serial.println("I set to " + String(itime/1000,2) + " ms");
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
    GetPID();
    //OUT0 to the piezo, OUT1 to the current controller
    analog.write(ToBits(PI_out+lock_point_offset),ToBits(-1.0*(pterm_current/pterm_piezo)*P_out-1.0*fterm*PI_out),true);

  /* AUTO-RELOCK */
    if(accumulator>50 || accumulator<-50) {
      Serial.println("Out of lock!");
      loop_counter = 0;
      accumulator = 0;
      //offset = PI_out;
      GetSetPoint();
      set_point = lock_point;
      // Scan once more until lock_point_time so system is in the capture range:
      ramp_reset_time = micros();
      do {
        current_time = micros(); //time in mircoseconds
        ramp_time = current_time-ramp_reset_time;
        analog.write(ToBits(RampOut(ramp_time)),ToBits(0),true);
      } while(ramp_time<lock_point_time);
      Serial.println("Relocked");
      Serial.print("lock-point: ");
      Serial.print(lock_point);
      Serial.print(" V   lock-point time: ");
      Serial.print(lock_point_time/1000.);
      Serial.println(" ms");
    }
    
    /*Write to serial for data logging in python */
    if(loop_counter%10000 == 0) {
      unsigned long current_time = millis();
      Serial.println(PI_out,4);
    } 
  }

}
