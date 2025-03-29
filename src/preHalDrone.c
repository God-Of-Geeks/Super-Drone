/*
 * Pre-Hal drone firmware
 * @file preHalDrone_esp32.c
 * @author jeswin.mathew, srikar.amara
 * @version 0.1
 * @date 2025-03-14
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "driver/i2c.h"
#include "driver/ledc.h"
//Arduino libraries for the sensors
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>

//handles for tasks
TaskHandle_t sensorTask = NULL;
TaskHandle_t pwmSignalTask = NULL;

const float temperature;
const float humidity;
const float pressure;

//handle for queues
QueueHandle_t bmeSensorQueue = NULL;
QueueHandle_t mpu6050Queue = NULL;

#define SEALEVELPRESSURE_HPA (1013.25)

//BME280 humidity sensor (I2C)
Adafruit_BME280 bme;

// Structure to hold MPU6050 sensor data
typedef struct {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
} mpu6050Data_t;


// Task to handle MPU6050 sensor readings
void MPU6050_SensorTask(void *pvParameters) {
  Serial.println("Starting MPU6050 Sensor Task on Core:");
  Serial.println(xPortGetCoreID());
  // Initialize I2C for MPU6050
  Wire.begin(21, 22); // SDA, SCL pins for ESP32
  Wire.setClock(400000); // Set I2C clock speed to 400kHz

  // Initialize MPU6050
  Adafruit_MPU6050 mpu;
  while (1) {

    //get sensor data and save into variables
    float accelX = mpu.readAccelerometerX();
    float accelY = mpu.readAccelerometerY();
    float accelZ = mpu.readAccelerometerZ();
    float gyroX = mpu.readGyroscopeX();
    float gyroY = mpu.readGyroscopeY();
    float gyroZ = mpu.readGyroscopeZ();

      // Log sensor data
      Serial.print("MPU6050 Sensor Data: ");
      Serial.print("Accel X: ");
      Serial.println(accelX);
      Serial.print("Accel Y: ");
      Serial.println(accelY);
      Serial.print("Accel Z: ");
      Serial.println(accelZ);
      Serial.print("Gyro X: ");
      Serial.println(gyroX);
      Serial.print("Gyro Y: ");
      Serial.println(gyroY);
      Serial.print("Gyro Z: ");
      Serial.println(gyroZ);

      // Send data to queue
      mpu6050Data_t sensorData = {accelX, accelY, accelZ, gyroX, gyroY, gyroZ};
      if (xQueueSend(mpu6050Queue, (void *)&sensorData, pdMS_TO_TICKS(10)) != pdPASS) {
          Serial.println("Failed to send data to MPU6050 queue");
      }

      // Delay before next reading
      vTaskDelay(pdMS_TO_TICKS(1000)); // Read every 1 second
  }
}

// Task to handle BME280 sensor readings
void BME280_SensorTask(void *pvParameters) {
  Serial.println("Starting BME280 Sensor Task on Core:");
  Serial.println(xPortGetCoreID());

  while (1) {
    // Read sensor data
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure();

    // Log sensor data
    Serial.println("Core 0 processing BME280 data");
    Serial.print("Temperature = ");
    Serial.print(1.8 * temperature + 32); // Convert to Fahrenheit
    Serial.println(" *F");

    Serial.print("Pressure = ");
    Serial.print(pressure / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    // Send data to queue
    xData_t sensorData = {temperature, humidity, pressure};
    if (xQueueSend(bmeSensorQueue, (void *)&sensorData, pdMS_TO_TICKS(10)) != pdPASS) {
      Serial.println("Failed to send data to BME280 queue");
    }

    // Delay before next reading
    vTaskDelay(pdMS_TO_TICKS(1000)); // Read every 1 second
  }
}

void getSensorData(void* pvParameters) {
  Serial.println("Getting sensor data from BME280 and MPU6050");
  // Create tasks for both sensors
  xTaskCreatePinnedToCore(BME280_SensorTask, "BME280_SensorTask", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(MPU6050_SensorTask, "MPU6050_SensorTask", 2048, NULL, 1, NULL, 1);
}

// Function to initialize PWM for the drone's motors
void initializePWM() {
  Serial.println("Initializing PWM signal for motors...");

  // Configure PWM channels for 4 motors
  for (int i = 0; i < 4; i++) {
      ledc_channel_config_t pwmChannel = {
          .gpio_num = 27++, //Use GPIO 27 and increment for each motor
          .speed_mode = LEDC_HIGH_SPEED_MODE,
          .channel = i,
          .timer_sel = LEDC_TIMER_0,
          .hpoint = 0
      };
      ledc_channel_config(&pwmChannel);
  }

  // Notes on PWM frequency and resolution:
  // - Duty cycle of  motor based on PWM resolution used (duty_resolution)
  // - duty cycle (%) = ((duty / (2^resolution)) * 100)

  // Configure the timer for PWM
  ledc_timer_config_t pwmTimer = {
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = LEDC_TIMER_10_BIT, // adjust based on duty cycle for ESCs
      .freq_hz = 50, // 50Hz for ESCs, adjust as needed
      .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&pwmTimer);

  Serial.println("PWM initialization complete.");
}

void initializeHardWare() {
  Serial.println("Initializing hardware...");

  // Initialize BME280 queue
  bmeSensorQueue = xQueueCreate(10, sizeof(xData_t));
  if (bmeSensorQueue == NULL) {
      Serial.println("Failed to create BME280 queue");
  }

  // Initialize MPU6050 queue
  mpu6050Queue = xQueueCreate(10, sizeof(mpu6050Data_t));
  if (mpu6050Queue == NULL) {
      Serial.println("Failed to create MPU6050 queue");
  }

  // Initialize PWM for motors
  initializePWM();

  // Call getSensorData to start sensor tasks
  getSensorData();
}

void setup() {
  Serial.begin(9600);

  Serial.print("Initializing hardware!!!");
  initializeHardWare();

  //change based on data receieved
  bmeSensorQueue = xQueueCreate(10, sizeof( &xData_t ));
  if(DataQueue == NULL){
     Serial.println("Queue was not created and must not be used");
   }

  bool status;
  status = bme.begin(0x76); //default address is 0x77
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  //TODO: Experiment on assigning sensors to core 0 and motor control to core 1
  //xTaskCreatePinnedToCore(getSensorData, "getSensorData", 960, NULL, 1, &sensorTask, 0);
  //xTaskCreatePinnedToCore(motorGenerateSignal, "motorGenerateSignal", 960, NULL, 1, &sensorTask, 0);

  Serial.println();
}

void loop() { 
  //Logic for reading sensor data done in tasks.
  //Nothing to do here maybe
}