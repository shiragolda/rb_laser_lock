# rb_laser_lock
Arduino servo to lock the frequency of the Rb laser

laser_frequency_servo contains code to use an arduino uno or chipkit uc32 to provide PI^2 feedback 
for locking the 780 nm laser to the Rb87 F=2 DAVLL signal. 
- PI^2 feedback is sent to the laser piezo
- P feedback + some feedforward of PI^2 is send to the laser current
Error and correction signals are broadcast over the computer's serial port. 

rb_laser lock contains code to publish the error and correction signals from the serial port to a zeroMQ socket. 
