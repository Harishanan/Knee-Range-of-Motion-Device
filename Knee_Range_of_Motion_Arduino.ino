//All right reserves to Harishanan Thevarajah.

#include <WiFi.h>         //library used to connect to Wi-Fi network
#include <HTTPClient.h>   //library used to send data to local host server(XAMPP in here)
#include <MPU9250_WE.h>   //library used to access MPU9250
#include <Wire.h>         //libary used to access I2C communication

//Initialise MPU9250 sensor at I2C address 0x68.
MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

//Define URL address of webserver, with appropriate computer ip address, project folder and php file name
String URL ="http://192.168.142.5/mpu9250_1/test_2.php";

//Define the name and password of Wi-Fi used to connect
const char* ssid = "Easy"; 
const char* password = "Harish123"; 

//Declare calibrated Accelerometer variables and Pitch and Roll angles
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

//Declare calibrated Gyroscope variables
float GyroX,GyroY,GyroZ;

//Define a prediction of initial Kalman Angle 
float KalmanAngleRoll=0; 
float KalmanAnglePitch=0;

//Define a prediction of initial uncertainty
float KalmanUncertaintyAngleRoll=2.0*2.0;
float KalmanUncertaintyAnglePitch=2.0*2.0;

//Initialising the output of a kalman filter by an array
float Kalman1DOutput[]={0,0};

  //Function that calculates the predicted angle and uncertainty using Kalman equation
  void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  //Predicting current state of the system
  KalmanState=KalmanState+0.1*KalmanInput;

  //Calculating uncertainty of thye prediction
  KalmanUncertainty=KalmanUncertainty + 0.1 * 0.1 * 4 * 4;

  //Caluclating Kalman gains from uncertainties on predictions and measurements
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 2 * 2);

  //Updating predicted state of the system with the measurement of state and Kalman gain
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);

  //Updating uncertatinty of the predicted system
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;

  //Assigning the outputs to Kalman Filter array
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void setup() {
  //Setting up serial communication between ESP32 and computer(used only for debugging purpose)
  Serial.begin(115200);                               

  //Access MPU9250 using I2C connection
  Wire.begin();

  //Ensure MPU9250 available
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  Serial.println("Position your MPU9250 flat and don't move it - calibrating...");
  delay(1000);

  //Do auto off set on each axis
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  //Configure accelerometer ouput scale range, here 4g has been chosen
  myMPU9250.setSampleRateDivider(0);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  
  //Switch on Digitial Low Pass Filter
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);

  //Configure gyroscope ouput scale range, here 250dps has been chosen
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

  //Switch on Digitial Low Pass Filter
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

  //Establishing Wi-Fi connection, by calling Wi-Fi function
  connectWiFi();  
}

//Function to update latest caluclated Kalman outputs
//This function will be called in the void loop to iterate
void kalman_readings(){
  //Calulating the rotation rates of gyroscope
  gyro_signals();
  
  //Iteration for the roll angle with Kalman Filter, called in void loop
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, GyroX, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0];              //Updating the latest Kalman roll outputs to the array
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];   //Updating latest Kalman roll uncertainty outputs to the array

  //Iteration for the pitch angle with Kalman Filter, called in void loop
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, GyroY, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0];             //Updating the latest Kalman pitch outputs to the array
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];  //Updating latest Kalman pitch uncertainty outputs to the array
 }


void gyro_signals() {
  //Read the raw measurements from accelerometer sensor
  xyzFloat accRaw = myMPU9250.getAccRawValues();
  
  //Convert accelerometer raw measurement value to physical value (g)
  xyzFloat accCorrRaw = myMPU9250.getCorrectedAccRawValues();
  
  //Provide calibrated accelerometer values (g)
  xyzFloat gValue = myMPU9250.getGValues();

  //Assign the calibrated values to declared accelerometer global variables
  AccX = gValue.x;
  AccY = gValue.y;
  AccZ = gValue.z;

  //Read the raw measurements from gyroscope sensor
  xyzFloat gyrRaw = myMPU9250.getGyrRawValues();

  //Convert gyroscope raw measurement value to physical value (dps)
  xyzFloat corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();

  //Provide calibrated gyroscope values (dps)
  xyzFloat gyr = myMPU9250.getGyrValues();

  //Assign the calibrated values to declared accelerometer global variables
  GyroX = gyr.x;
  GyroY = gyr.y;
  GyroZ = gyr.z;

  //Calculating roll and pitch angle using trignometry
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / 3.142);
  AnglePitch = atan(-AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / 3.142);
}


void loop() {
   if(WiFi.status() != WL_CONNECTED) { 
    connectWiFi();
  }

  //Iterating the Kalman Filter to find latest outputs
  kalman_readings();

  //Turns on the inbuilt LED to let user know a data is transfered
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  //Printing Kalaman Angle Pitch and Roll values to verify with sending data in Serial monitor
  Serial.print("X axis [deg] = ");
  Serial.print(KalmanAnglePitch);
  Serial.print(", Y axis [deg] = ");
  Serial.println(KalmanAngleRoll);

  //Holding the latest Kalman roll and pitch angles in a variable to send to MySQL using HTTP POST
  String postData = "xAngle=" + String(KalmanAnglePitch) + "&yAngle=" + String(KalmanAngleRoll); 

  //Creating http object for HTTPClient class
  HTTPClient http;

  //Esablishing a connection to defined XAMPP server url.
  http.begin(URL);

  //Defining the header to Server to understand the sendind data type
  http.addHeader("Content-Type", "application/x-www-form-urlencoded"); 

  //Store the status of HTTP data sending reply from server(200:successfull HTTP Post, -1:Network related issue, -5:MySQL related issue)
  int httpCode = http.POST(postData); 

  //Displays the response message from server in reply of HTTP send data
  String payload = http.getString(); 

  //Print the above values in Serial Monitor, for testing verification.
  Serial.print("URL : "); Serial.println(URL); 
  Serial.print("Data: "); Serial.println(postData); 
  Serial.print("httpCode: "); Serial.println(httpCode); 
  Serial.print("payload : "); Serial.println(payload); 
  Serial.println("--------------------------------------------------");

  //turns off the LED after each data transfered
  digitalWrite(13, LOW);

  //delay(100);
}


//Function for Arduino WiFi Station Mode
void connectWiFi() {
  //Turn off the Wi-Fi, to disable the existing Wi-Fi mode
  WiFi.mode(WIFI_OFF);
  delay(1000);

  //Turn on the Wi-Fi station mode
  WiFi.mode(WIFI_STA);          

  //Establish Wi-Fi connection to define Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");

  //Check ESP32 is connected to define Wi-Fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  //Print the Wi-Fi connection established message for debugging purpose
  Serial.print("connected to : "); Serial.println(ssid);
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
}
