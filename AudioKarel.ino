/*
  SD card Datalogger for Arduino Nano

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2


 Created by Forsat Ingenieria S.A.S

*/

#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <Timer.h>

// Macros
#define chipSelect 4
// config to arduino pro mini

#define sensor1Pin A6
#define sensor2Pin A7
#define sensor3Pin A0
#define sensor4Pin A1
#define sensor5Pin A2
#define sensor6Pin A3
#define sensor7Pin A4
#define sensor8Pin A5

#define resetPin 2
#define timeToUploadDataToCloud 120000
#define writingOnSDCardState 1
#define readingKarelDataState 2
#define standByState 3
#define noTasksPending 0 
#define sendingPackageDataToWifiTask 1
#define sensorFeedVoltaje 5
// Global Variables
 
// Change Serial Pins Because Bridge Uses 0 and 1 as Rx and Tx
SoftwareSerial mySerial(8, 7); // RX, TX
   
// Others 
 bool shouldStartReading=false;
 byte globalState=standByState;
 unsigned long milliSeconds=0;
 unsigned int index=0;
 int firstScaler=0;
 int numberOfRawsInExcel=30000; 
 int indexToChangeFileName=0;
 int indexReadFromEEprom=0;
 int sensor1Value=0;
 int sensor2Value=0;
 int sensor3Value=0;
 int sensor4Value=0;
 int sensor5Value=0;
 int sensor6Value=0;
 int sensor7Value=0;
 int sensor8Value=0;
 
 byte karelData1=0;
 int karelData2=0;
 long karelData3=0;
 word karelData4=0;
 byte karelData5=0;
 unsigned int bufferSize=30;
 unsigned int characterOfEnd=69;
 unsigned int characterOfStart=83;
 volatile char pendingTasks[10];
 char myBuffer[50];
 char numberOfFile[5];     
 // Timer Variables
 Timer dataUploadTimer;                              

const int outPin = 9; // PWM Audio Port out

const unsigned int SAMPLE_RATE = 44100; // Hz
const int SAMPLES = 512 ; // Num of Samples (128,256,512,1024)
const int REPEAT = 3;  // prolong the signal
const int OFFSET = 60;  // 60 un-usable bin on the spectrum's low end
const double FREQ_RES = (double) SAMPLE_RATE / SAMPLES; // 86.13 frq per bin
const int DURATION = (int) REPEAT * (1000 / FREQ_RES); // about 35ms
String data;

void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  // Welcome Message
  Serial.println("Filesystem datalogger\n");

  // Initialize and Set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

  // Initialize timer methods
  
  // Set timer to Upload Sensor Data to Cloud every X time
  dataUploadTimer.every(timeToUploadDataToCloud, timeInterruptHandling);       
  
  // Flag which turn the LED on to check when it's been already set up
  //digitalWrite(13, HIGH);  
  
  // Initialize tasks vector
  
  // Send Package Data Task
  pendingTasks[0]=noTasksPending; 
  // Send Reset Command Task
  pendingTasks[1]=noTasksPending;  
  
  // Initialize IRQ in digital pin #D2 and prepare it as well for digital reading
  pinMode(resetPin,INPUT);
  attachInterrupt(digitalPinToInterrupt(resetPin), eventIRQHandling, RISING);

pinMode( outPin, OUTPUT ); 
timeInterruptHandling();
                                   
}

void loop() {
  
   // Check if there is any pending task
   checkForPendingTasks();

     
   // Update Timers
   updateTimers();
  
   // Write dataString on the SD CARD on Interrupt Serial Handling Event
   while(mySerial.available()>0){
      
      // Read the Serial 
      myBuffer[index]=(mySerial.read());
      
      // Check if should start reading
      if (myBuffer[index]==characterOfStart)
      {
        shouldStartReading=true;
      }
      
      // Go to save the serial data into a vector
      if (shouldStartReading==true)
      {
        saveSerialData();
      }
      
   }
    if(mySerial.available()==0){
        //Serial.println("NoData");
    }

}

// Method to update my timers
void updateTimers(){
  
  dataUploadTimer.update();

}

// Method to check for pending tasks
void checkForPendingTasks(){
  
  if ((pendingTasks[0]==noTasksPending) && (pendingTasks[1]==noTasksPending))
  {
    //Serial.println("NoTasks");
    // Do nothing!
  }
  else{
    
    // Check if it's needing to send data to wifi task and its state allows it
    if ((pendingTasks[0]==sendingPackageDataToWifiTask) && (globalState!=writingOnSDCardState))
    {
      sendPackageDataToWifiModule();

      // Set this pending task as noPendingTask
      pendingTasks[0]=noTasksPending;
    }
  }
}

// Method to save data into a buffer

void saveSerialData(){
  
   //Serial1.print(myBuffer[index]);
   
   // Place the new data in a new position of the vector every time that a byte is in the buffer
   index++;
   
   // Check if data received is completed
   checkIfDataIsComplete();
   
   // Delay to Wait for the Serial Communication
   delay(1);
  
}

// Check if the data is complete to start writing on the SD
void checkIfDataIsComplete(){
  
   // Compare if Character Message has the appendix, the end, and the number of characters indicated
    if ((myBuffer[0]==characterOfStart) && (myBuffer[index-1]==characterOfEnd) && (index==bufferSize))
   {
     
    Serial.println("Cadena correcta");
    
    // Execute all process to write the message
    executeProcessToWriteMessagesOnSD();
       
  }
  
  // Check if should discard the message and reset the buffer
  if (index>bufferSize)
  {
    // Discard Message
    discardMessage();
  }
  
}

// Method which orders a way of execution in the software for saving on SD 
void executeProcessToWriteMessagesOnSD(){

  // Stop Listening to serial to give a time to my processor to write on SD
  mySerial.end();
  
  // Tell to the program that it's busy writing on SD Card
  changeState(writingOnSDCardState);

  // Read sensors values from Analog Inputs
  readSensorsValues();
  
  // Prepare the data to write on SD and Upload to CLoud
  handlingDataReceived();
  
  // Restart some values needed to enable reading from serial
  prepareToReadSerialAgain();
  
}

// Method to change the current state of micro-controller
void changeState(byte state){
  
  globalState=state;
}

// Method to restart the Serial Reading

void prepareToReadSerialAgain(){

  // Switch flag to control the reading
  shouldStartReading=false;
  

  // Method to clean buffers 
  cleanBuffers();

  // Tell to the program that it's not writing on SD Card anymore
  changeState(readingKarelDataState);
  
  // Restart the Serial Listening 
  mySerial.begin(9600);

}

// Method to Discard Message

void discardMessage(){
  
  Serial.println("ERRONEA");
    
  // Stop the Serial Communication
  mySerial.end();
    
  // Clean the buffer
  for (int h=0;h<index;h++)
  {
    myBuffer[h]='/0';
  }
    
  // Restart the index
  index=0;
    
  // Restart the Serial Communication
  mySerial.begin(9600);
  
}
// Classify the data to write on SD
void handlingDataReceived(){

   data = "?";
   // Clean the Buffer and Pass the info to another Vector
   for (int i=0;i<index;i++)
   {
     
    // Method to Write a Karel data on the SD byte per byte
    writeKarelDataOnSDWithComma(myBuffer[i]);
       
    // Check if Character Received Was ACK, which is the last character Karel Serial data and it's 6 in DECIMAL format. If it does, we continue to write the following stuff on SD
    if ((myBuffer[i]==characterOfEnd) && (i==bufferSize-1)){
          
      // Write Sensors Values 
      sendTone();
    
      // Hold Bytes Values From Karel Data
      holdKarelData();
      
    }
     
   }  
}

// Method to write data on SD when called
void writeKarelDataOnSDWithComma(char byteWithCommaReceived){  
  
 
    // Flag to physically know if there is any mistake or if it's writing correctly
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

    //Serial.println(byteWithCommaReceived);
    byte karelInt = byteWithCommaReceived;
    // Write Data on SD
    data = data + String(karelInt)+",";
    // Write Separation among bytesF
      
    // Flag to physically know if there is any mistake or if it's writing correctly
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    
  
}

// Method to write Sensor values on SD from Analog Reads
void sendTone(void){
 
  // Write Sensors Values Separated By Commas
  data = data + String(sensor1Value, DEC)+","+String(sensor2Value, DEC)+","+String(sensor3Value, DEC)+","+String(sensor4Value, DEC)+","+String(sensor5Value, DEC)+","+String(sensor6Value, DEC)+"@";
  //Serial.println("");
  
  int i=0;
  do { 
  //tone(outPin, frq[i]);
  tone(outPin,((5 *(( data.charAt(i))-45) + OFFSET) * FREQ_RES));
  delay(DURATION); 
  noTone(outPin);
  }
  while (data.charAt(i++) != '@'); 
  Serial.println("escribiendo datos..." + data);
  data ="";
    
}


// Method to read sensors data from analog inputs
void readSensorsValues(void){
  
  // Read & Calculate the real temperature value according to ADC Scale read from analog inputs and Alloc the values
  sensor1Value=calculateRealTemperatureValue(analogRead(sensor1Pin));
  sensor2Value=calculateRealTemperatureValue(analogRead(sensor2Pin));
  sensor3Value=calculateRealTemperatureValue(analogRead(sensor3Pin));
  sensor4Value=calculateRealTemperatureValue(analogRead(sensor4Pin));
  sensor5Value=calculateRealTemperatureValue(analogRead(sensor5Pin));
  sensor6Value=calculateRealTemperatureValue(analogRead(sensor6Pin));
  sensor7Value=calculateRealTemperatureValue(analogRead(sensor8Pin));
  sensor8Value=calculateRealTemperatureValue(analogRead(sensor7Pin));
  //sensor3Value=analogRead(sensor3Pin);

  Serial.println("Analog Channels Reading");
  Serial.println("Analog Channels Reading");
  Serial.println(analogRead(sensor1Pin));
  Serial.println(analogRead(sensor2Pin));
  Serial.println(analogRead(sensor3Pin));
  Serial.println(analogRead(sensor4Pin));
  Serial.println(analogRead(sensor5Pin));
  Serial.println(analogRead(sensor6Pin));
  Serial.println(analogRead(sensor7Pin));
  Serial.println(analogRead(sensor8Pin));

 // Serial.println("Valor de Sensores");
  
  // Real time debugging printing
  //Serial.println(sensor1Value);
  //Serial.println(sensor2Value);

}

// Method to hold the Karel data needed in global variables
void holdKarelData(){

  // Assign to global variables
  
  // Outputs Register
  karelData1=myBuffer[1];
  // Temperature Register Signed Conversion
  byte temporalKarelData2=myBuffer[4];
  if(temporalKarelData2<=50){
    karelData2=temporalKarelData2;}
  else{
    karelData2=(temporalKarelData2-255);}
  // AR Register Signed Conversion
  word temporalKarelData3=word(myBuffer[5],myBuffer[6]);
  if(temporalKarelData3<32000){
    karelData3=word(myBuffer[5],myBuffer[6]);}
  else{
    karelData3=(temporalKarelData3-65535);}
  // T Accumulated Register
  karelData4=word(myBuffer[15],myBuffer[16]);
  // Set Level Register
  karelData5=myBuffer[2];
  
  Serial.println("Valor de Karel Data");
  
  // Real time debugging 
  Serial.println(karelData1);
  Serial.println(karelData2);
  Serial.println(karelData3);
  Serial.println(karelData4);
  Serial.println(karelData5);
  
}

// Method to Calculate and return the real temperature value according to ADC Scale Value
double calculateRealTemperatureValue(double ADCValue){
  
  // Process to calculate the value in temperature
  double measuredVoltage=sensorFeedVoltaje*ADCValue/1024;
  double resistenceValue=(10)*(sensorFeedVoltaje-measuredVoltage)/measuredVoltage;
  //double temperatureValue=((-20.63)*log(resistenceValue))+45.7;
  double temperatureValue=((-0.0983)*pow(log(resistenceValue),3))+((1.5293)*pow(log(resistenceValue),2))+((-25.848)*log(resistenceValue))+49.746;
  
  
  // Return Value
  return temperatureValue;
}

// Method to send data to modulo wifi
void sendPackageDataToWifiModule(){

  // Read Sensor Values Before Uploading
  readSensorsValues();
  

  // Debug Comments
  Serial.println("Sending Data to Wifi Module");
  //handlingDataReceived();
  // Send Karel Data Package to Wifi Module in following order: Outputs Register-Temperature Register-AR Register-T Accumulated Register-Sensor1-Sensor2
  
  // Build a string as needed to send the data package to wifi module
  // 36 data 30 karel 6 analog
  data = "?0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,";

  // Send Data by Serial Interface
    sendTone();

  // Debugging String
  
  
}

// Method to clean the buffers and get it ready to keep receiving data
void cleanBuffers(){
  
  // Run the vector
   for (int j=0;j<index;j++)
   {
     // Clean the buffer
     myBuffer[j]='/0';
   }    
   
  // Set the index of the vector to the position 0
  index=0;
}


// Method which IRQ interrupt lands
void eventIRQHandling(){
  
  // Turns interruption down to avoid getting in here repeatedly for just one push 
  detachInterrupt(0);

  // Accumulate a pending task for when it's available to do it
  
}

// Method to update all data to cloud
void timeInterruptHandling(){ 

  // Flag to notify the software that something is pending
  pendingTasks[0]=sendingPackageDataToWifiTask;
    
  Serial.println("Time to Send Data Package");
  Serial.println("Time to Send Data Package");
  
}
