#define __FREERTOS 1

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// our libs
#include <BMP.h>
#include <GPS.h>
#include <ICM20649.h>
#include <LIS3MDL.h>
#include <pindefs.h>

// defining sensors 
BMP bmpA(Wire, 0x77);
BMP bmpB(Wire1, 0x77);
ICM20649 icm(Wire);
LIS3MDL lis(Wire);
GPS gps(Serial1, GPS_TX, GPS_RX);


// --------------------
// tracks current state
// --------------------
enum class FlightState {
    START,          
    GROUND_MODE,          
    FLIGHT_MODE,
    DESCENT_MODE,
    CHILL,
    FAILED
};

// --------------------
// Shared resource
// --------------------
struct vector3 {
  float x;
  float y;
  float z;
};
struct SharedData
{
  float BMPA_Temperature;
  float BMPA_Pressure;
  float BMPA_Altitude;
  float BMPA_Velocity;
  float BMPB_Temperature;
  float BMPB_Pressure;
  float BMPB_Altitude;
  float BMPB_Velocity;
  vector3 ICM_Accel;
  float ICM_Velocity_Z; //added
  vector3 ICM_Gyro;
  vector3 ICM_Theta;
  float ICM_Temperature;
  vector3 LIS_Mag;
  float LIS_Temperature;
  float GPS_Latitude;
  float GPS_Longitude;
  FlightState current_state = FlightState::START;
};
SemaphoreHandle_t dataMutex;
SharedData sharedData;

volatile int globalCounter = 0;
SemaphoreHandle_t counterMutex;

// --------------------
// Task prototypes
// --------------------
void task1(void* param);
void task2(void* param);
void task3(void* param);
void task4(void* param);
void task5(void* param);
void get(void* param);
void difint(void* param);

void runStateMachine(SharedData snapshot); //method prototype

FlightState currentState = FlightState::START;  //IF NO SPACE CAN BE DELETED, NOT NECESSARY

//IMPORTANT IMPORTANT IMPORTANT
//FUNCTIONS TO IMPLEMENT:
//1. initialize()
//2. writeToSDCard(SharedData snapshot)
//3. bmp.differentiate()
//4. icm.integrate()
//some comments for the integrate function, currently integration is being done in getvelocity
//Shift the integration in getvelocity to a seperate function integrate which has been used
//in the difint function
//5. icm.tipover(SharedData snapshot) - returns true or false
//6. deployMain()
//7. IMPLEMENT ICM VELOCITY Z, in ICM it should be icm.getVelocityZ()
//8. CHERUKU'S TASK - implement icm.getAngleX(),icm.getAngleY(),icm.getAngleZ()
//9. deployBackup()
//10. for all the serial prints in every task, they're within the global lock code, i think
//    we should shift it to outside the global lock to save time, but we can see that later
//
//TASKS LEFT:
//implement FAIL mode too, with the proper checks if a sensor fails
//Chigga communication, starting the threads, and thread priorities also have to be set 


void setup() {
  Serial.begin(115200);
  Wire.setSDA(SDA_A);
  Wire.setSCL(SCL_A);
  Wire1.setSDA(SDA_B);
  Wire1.setSCL(SCL_B);
  while (!Serial);

  Serial.println("Starting FreeRTOS with shared memory...");

  // Create mutex
  counterMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  if (dataMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (1);
  }

  // Create tasks
  xTaskCreate(task1, "BMPA", 1024, NULL, 1, NULL);
  xTaskCreate(task2, "BMPB", 1024, NULL, 1, NULL);
  xTaskCreate(task3, "ICM", 1024, NULL, 1, NULL);
  xTaskCreate(task4, "LIS", 1024, NULL, 1, NULL);
  xTaskCreate(task5, "GPS", 1024, NULL, 1, NULL);
  xTaskCreate(get, "read snapshot", 1024, NULL, 1, NULL);
  xTaskCreate(difint, "integrate", 1024, NULL, 1, NULL);

  // init senors
  bmpA.begin();
  bmpB.begin();
  icm.begin();
  lis.begin();
  gps.begin();
  

  //CHECK INITIALIZATION OF CHIGGA AND ONLY THEN SET
  if(initialize()){   //implement initialize to check if everything is running
    currentState = FlightState::GROUND_MODE;
    sharedData.current_state = FlightState::GROUND_MODE;

  }
  else{
    Serial.println("INITIALIZATION FAILED, SWITCHING TO CHIGGA");
    currentState = FlightState::FAILED;
  }
  
}

void loop() {
  // Not used
}

// --------------------
// Task 1 : BMPA
// --------------------
void task1(void* param) {
  for (;;) {
    bmpA.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.BMPA_Temperature = bmpA.getTemperature();
      sharedData.BMPA_Pressure = bmpA.getPressure();
      sharedData.BMPA_Altitude = bmpA.getAltitude();
      sharedData.BMPA_Velocity = bmpA.getVelocity();
      // inline prinitng
      Serial.print("BMPA Temp: ");
      Serial.print(sharedData.BMPA_Temperature);
      Serial.print(" C, Pressure: ");
      Serial.print(sharedData.BMPA_Pressure);
      Serial.print(" hPa, Altitude: ");
      Serial.print(sharedData.BMPA_Altitude);
      Serial.print(" m, Velocity: ");
      Serial.print(sharedData.BMPA_Velocity);
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// --------------------
// Task 2 : BMPB
// --------------------
void task2(void* param) {
  for (;;) {
    bmpB.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.BMPB_Temperature = bmpB.getTemperature();
      sharedData.BMPB_Pressure = bmpB.getPressure();
      sharedData.BMPB_Altitude = bmpB.getAltitude();
      sharedData.BMPB_Velocity = bmpB.getVelocity();
      Serial.print("BMPB Temp: ");
      Serial.print(sharedData.BMPB_Temperature);
      Serial.print(" C, Pressure: ");
      Serial.print(sharedData.BMPB_Pressure);
      Serial.print(" hPa, Altitude: ");
      Serial.print(sharedData.BMPB_Altitude);
      Serial.print(" m, Velocity: ");
      Serial.print(sharedData.BMPB_Velocity);
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1500));
  }
}

// --------------------
// Task 3 : ICM20649
// --------------------
void task3(void* param) {
  for (;;) {
    icm.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.ICM_Accel.x = icm.getAccelX();
      sharedData.ICM_Accel.y = icm.getAccelY();
      sharedData.ICM_Accel.z = icm.getAccelZ();
      sharedData.ICM_Gyro.x = icm.getGyroX();
      sharedData.ICM_Gyro.y = icm.getGyroY();
      sharedData.ICM_Gyro.z = icm.getGyroZ();
      sharedData.ICM_Temperature = icm.getTemperature();
      sharedData.ICM_Velocity_Z = icm.getVelocityZ(); //implement
      sharedData.ICM_Theta.x = icm.getAngleX(); //implement
      sharedData.ICM_Theta.y = icm.getAngleY(); //implement
      sharedData.ICM_Theta.z = icm.getAngleZ(); //implement
      Serial.print("ICM Accel: (");
      Serial.print(sharedData.ICM_Accel.x);
      Serial.print(", ");
      Serial.print(sharedData.ICM_Accel.y);
      Serial.print(", ");
      Serial.print(sharedData.ICM_Accel.z);
      Serial.print(") m/s^2, Gyro: (");
      Serial.print(sharedData.ICM_Gyro.x);
      Serial.print(", ");
      Serial.print(sharedData.ICM_Gyro.y);
      Serial.print(", ");
      Serial.print(sharedData.ICM_Gyro.z);
      Serial.print(") deg/s, Temp: ");
      Serial.print(sharedData.ICM_Temperature);
      Serial.println(" C");
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// --------------------
// Task 4 : LIS3MDL
// --------------------
void task4(void* param) {
  for (;;) {
    lis.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.LIS_Mag.x = lis.getMagX();
      sharedData.LIS_Mag.y = lis.getMagY();
      sharedData.LIS_Mag.z = lis.getMagZ();
      sharedData.LIS_Temperature = lis.getTemperature();
      Serial.print("LIS Mag: (");
      Serial.print(sharedData.LIS_Mag.x);
      Serial.print(", ");
      Serial.print(sharedData.LIS_Mag.y);
      Serial.print(", ");
      Serial.print(sharedData.LIS_Mag.z);
      Serial.print(") uT, Temp: ");
      Serial.print(sharedData.LIS_Temperature);
      Serial.println(" C"); 
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2500));
  }
}

// --------------------
// Task 5 : GPS
// --------------------
void task5(void* param) {
  for (;;) {
    gps.read();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      sharedData.GPS_Latitude = gps.getLatitude();
      sharedData.GPS_Longitude = gps.getLongitude();
      Serial.print("GPS Latitude: ");
      Serial.print(sharedData.GPS_Latitude);
      Serial.print(" deg, Longitude: ");
      Serial.print(sharedData.GPS_Longitude);
      Serial.println(" deg");
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// --------------------
// Task 6 : GET CURRENT READINGS
// --------------------
void get(void* param) {
    for (;;) {
        SharedData snapshot;

        // take global lock, copy everything, release immediately
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            snapshot = sharedData;
            xSemaphoreGive(dataMutex);
        }
        //now runs the ASM function
        runStateMachine(snapshot);
        vTaskDelay(pdMS_TO_TICKS(50)); //can decrease delay, or just set priority, use xTaskCreate then
    }
}

// --------------------
// Task 7 : INTEGRATION 
// --------------------
void difint(void* param) {
    
    for (;;) {
        //bmpB.differentiate();   - CHIGGA
        if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        bmpA.differentiate();   
        icm.integrate(); 
        xSemaphoreGive(dataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); //can decrease/increase delay, or just set priority, use xTaskCreate then
    }
}



// --------------------
// ASM CODE
// --------------------

//ASM VARIABLES
//change all the variables except count
float acc_threshold = 0; //parameters for main, for flight mode
float vel_threshold = 0;
int count_BMP = 0;
int count_ICM = 0;
float acc_threshold2 = 0;   //parameters for backup, threshold2 is for descent mode
float vel_threshold2 = 0;
int count_BMP2 = 0;
int count_ICM2 = 0;


void runStateMachine(SharedData snapshot){
    writeToSDCard(snapshot);
    if(currentState == FlightState::GROUND_MODE){
        if(snapshot.BMPA_Altitude>200||snapshot.ICM_Accel.z>acc_threshold){
            currentState = FlightState::FLIGHT_MODE;
            sharedData.current_state = FlightState::FLIGHT_MODE;
        } 
    }
    else if(currentState == FlightState::FLIGHT_MODE){
        if(icm.tipover(snapshot) == true){
            if(snapshot.BMPA_Velocity<=vel_threshold&&count_BMP>=5){
                deployMain();
                currentState = FlightState::DESCENT_MODE;
                sharedData.current_state = FlightState::DESCENT_MODE;
            }
            else if(snapshot.BMPA_Velocity<=vel_threshold){
                count_BMP++;
            }
            else if(snapshot.ICM_Velocity_Z<=vel_threshold&&count_ICM>=5){
                deployMain();
                currentState = FlightState::DESCENT_MODE;
                sharedData.current_state = FlightState::DESCENT_MODE;
            }
            else if(snapshot.ICM_Velocity_Z<=vel_threshold){
                count_ICM++;
            }
            else{
                return;
            }
        }
        else if(snapshot.BMPA_Velocity<=vel_threshold&&count_BMP>=5){
            if(snapshot.ICM_Velocity_Z<=vel_threshold&&count_ICM>=5){
                deployMain();
                currentState = FlightState::DESCENT_MODE;
                sharedData.current_state = FlightState::DESCENT_MODE;
            }
            else if(snapshot.ICM_Velocity_Z<=vel_threshold){
                count_ICM++;
            }
        }
        else if(snapshot.BMPA_Velocity<=vel_threshold){
            count_BMP++;
        }
    }
    else if(currentState == FlightState::DESCENT_MODE){
        if(icm.tipover(snapshot) == true){
            if(snapshot.BMPA_Velocity<=vel_threshold2&&count_BMP2>=5){
                deployBackup();
                currentState = FlightState::CHILL;
                sharedData.current_state = FlightState::CHILL;
            }
            else if(snapshot.BMPA_Velocity<=vel_threshold2){
                count_BMP2++;
            }
            else if(snapshot.ICM_Velocity_Z<=vel_threshold2&&count_ICM2>=5){
                deployBackup();
                currentState = FlightState::CHILL;
                sharedData.current_state = FlightState::CHILL;
            }
            else if(snapshot.ICM_Velocity_Z<=vel_threshold2){
                count_ICM2++;
            }
            else{
                return;
            }
        }
        else if(snapshot.BMPA_Velocity<=vel_threshold2&&count_BMP2>=5){
            if(snapshot.ICM_Velocity_Z<=vel_threshold2&&count_ICM2>=5){
                deployBackup();
                currentState = FlightState::CHILL;
                sharedData.current_state = FlightState::CHILL;
            }
            else if(snapshot.ICM_Velocity_Z<=vel_threshold2){
                count_ICM2++;
            }
        }
        else if(snapshot.BMPA_Velocity<=vel_threshold2){
            count_BMP2++;
        }
    }
}
