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
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

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

//Function to get BME pressure readings
void BME280_SensorReadings(void *pvParameters) {
    Serial.println(xPortGetCoreID());

    while (1) {

        if(xQueueSend( bmeSensorQueue, (void *) &pxPointerToxData_t, 10 ) != pdPASS )
        {
            Serial.println("Unable to pass messages to queue"); 
        }
        // Read sensor data
        Serial.println("Core 0 processing");
        temperature = bme.readTemperature();
        humidity = bme.readHumidity();
        pressure = bme.readPressure();

        // Convert temperature to Fahrenheit
        Serial.print("Temperature = ");
        Serial.print(1.8 * temperature + 32);
        Serial.println(" *F");
        
        Serial.print("Pressure = ");
        Serial.print(pressure / 100.0F);
        Serial.println(" hPa");

        Serial.print("Approx. Altitude = ");
        Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(" m");

        Serial.print("Humidity = ");
        Serial.print(humidity);
        // Delay before next reading
        vTaskDelay(pdMS_TO_TICKS(1000)); // Read every 1 second
    }
}

void getSensorData(void* pvParameters) {
    Serial.print("Getting sensor data from BME280");
    BME280_SensorReadings();
}

void initializeHardWare()
{
    //TODO: Add sensor failure recovery mechanisms, suspend tasks, etc
    //check for sesnsor transmitting address
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
  Serial.println("-- Test readings from sensors --");
  //assign core 0 for sensor readings and 1 for generating PWM signal
  xTaskCreatePinnedToCore(getSensorData, "getSensorData", 960, NULL, 1, &sensorTask, 0);
  xTaskCreatePinnedToCore(pwmSignalTask, "pwmSignalTask", 10000, NULL, 1, &pwmSignalTask, 1);

  Serial.println();
}

void loop() { 
  //Logic for reading sensor data done in tasks.
  //Nothing to do here maybe
}