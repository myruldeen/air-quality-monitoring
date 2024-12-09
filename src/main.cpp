#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
// Pin definitions
#define PMS_RX 16
#define PMS_TX 17
#define PMS_BAUD 9600

#define I2C_SDA 21 
#define I2C_SCL 22

// Task handles
TaskHandle_t pmsReadTaskHandle = NULL;
TaskHandle_t bmeReadTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;

// Queue handle for sensor data
QueueHandle_t pmsDataQueue;
QueueHandle_t bmeDataQueue;

// Mutex for serial communication
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t wireMutex;

// Hardware Serial for PMS sensor
HardwareSerial pmsSensor(2);
Adafruit_BME680 bme;

// Structure to hold PMS5003 data
struct PMS5003Data {
    uint16_t framelen;
    uint16_t pm10_standard, pm25_standard, pm100_standard;
    uint16_t pm10_env, pm25_env, pm100_env;
    uint16_t particles_03um, particles_05um, particles_10um;
    uint16_t particles_25um, particles_50um, particles_100um;
    uint16_t unused;
    uint16_t checksum;
    bool isValid;
};

// Structure to hold BME680 data
struct BME680Data {
    float temperature;
    float humidity;
    float pressure;
    float gas;
    bool isValid;
};

void resetPMSSerial() {
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Serial.println("Resetting PMS Serial...");
        pmsSensor.end();
        delay(100);
        pmsSensor.begin(PMS_BAUD, SERIAL_8N1, PMS_RX, PMS_TX);
        
        while(pmsSensor.available()) {
            pmsSensor.read();
        }
        delay(1000);
        
        xSemaphoreGive(serialMutex);
    }
}

bool readPMS5003(PMS5003Data &data) {
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }

    if (pmsSensor.available() < 32) {
        xSemaphoreGive(serialMutex);
        return false;
    }

    bool found_start = false;
    uint32_t start_time = millis();
    
    while (millis() - start_time < 1000 && !found_start) {
        if (pmsSensor.available() >= 2) {
            if (pmsSensor.read() == 0x42 && pmsSensor.peek() == 0x4D) {
                found_start = true;
                pmsSensor.read(); // consume 0x4D
            }
        }
    }

    if (!found_start) {
        xSemaphoreGive(serialMutex);
        return false;
    }

    uint8_t buffer[30];
    if (pmsSensor.readBytes(buffer, 30) != 30) {
        xSemaphoreGive(serialMutex);
        return false;
    }

    uint16_t calcChecksum = 0x42 + 0x4D;
    for (uint8_t i = 0; i < 28; i++) {
        calcChecksum += buffer[i];
    }

    xSemaphoreGive(serialMutex);

    uint16_t receivedChecksum = buffer[28] << 8 | buffer[29];
    if (calcChecksum != receivedChecksum) {
        return false;
    }

    // Parse data
    data.framelen = buffer[0] << 8 | buffer[1];
    data.pm10_standard = buffer[2] << 8 | buffer[3];
    data.pm25_standard = buffer[4] << 8 | buffer[5];
    data.pm100_standard = buffer[6] << 8 | buffer[7];
    data.pm10_env = buffer[8] << 8 | buffer[9];
    data.pm25_env = buffer[10] << 8 | buffer[11];
    data.pm100_env = buffer[12] << 8 | buffer[13];
    data.particles_03um = buffer[14] << 8 | buffer[15];
    data.particles_05um = buffer[16] << 8 | buffer[17];
    data.particles_10um = buffer[18] << 8 | buffer[19];
    data.particles_25um = buffer[20] << 8 | buffer[21];
    data.particles_50um = buffer[22] << 8 | buffer[23];
    data.particles_100um = buffer[24] << 8 | buffer[25];
    data.isValid = true;

    return true;
}

// Task to read PMS5003 sensor
void pmsReadTask(void *parameter) {
    static unsigned long failedAttempts = 0;
    PMS5003Data sensorData;

    while (1) {
        sensorData.isValid = false;
        
        if (readPMS5003(sensorData)) {
            failedAttempts = 0;
            xQueueSend(pmsDataQueue, &sensorData, 0);
        } else {
            failedAttempts++;
            if (failedAttempts >= 5) {
                resetPMSSerial();
                failedAttempts = 0;
            }
        }
        
        // Wait for 2 seconds before next reading
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Task to read BME680 sensor
void bmeReadTask(void *parameter) {
    BME680Data sensorData;
    
    while (1) {
        sensorData.isValid = false;
        
        if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (bme.performReading()) {
                sensorData.temperature = bme.temperature;
                sensorData.humidity = bme.humidity;
                sensorData.pressure = bme.pressure / 100.0;  // Convert to hPa
                sensorData.gas = bme.gas_resistance / 1000.0;  // Convert to kOhms
                sensorData.isValid = true;
                
                xQueueSend(bmeDataQueue, &sensorData, 0);
            }
            xSemaphoreGive(wireMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));  // Read every 2 seconds
    }
}

// Task to display sensor data
void displayTask(void *parameter) {
    PMS5003Data pmsData;
    BME680Data bmeData;
    static unsigned long readCount = 0;

    while (1) {
        bool pmsReceived = false;
        bool bmeReceived = false;
        
        // Try to receive data from both sensors
        pmsReceived = (xQueueReceive(pmsDataQueue, &pmsData, pdMS_TO_TICKS(5000)) == pdTRUE);
        bmeReceived = (xQueueReceive(bmeDataQueue, &bmeData, pdMS_TO_TICKS(100)) == pdTRUE);
        
        if (pmsReceived || bmeReceived) {
            readCount++;
            
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                Serial.printf("\nRead #%lu\n", readCount);
                
                if (pmsReceived) {
                    Serial.println("\nPM Concentrations (Standard):");
                    Serial.printf("PM1.0: %u μg/m3\n", pmsData.pm10_standard);
                    Serial.printf("PM2.5: %u μg/m3\n", pmsData.pm25_standard);
                    Serial.printf("PM10:  %u μg/m3\n", pmsData.pm100_standard);
                    
                    Serial.println("\nParticle counts per 0.1L:");
                    Serial.printf(">0.3um: %u\n", pmsData.particles_03um);
                    Serial.printf(">0.5um: %u\n", pmsData.particles_05um);
                    Serial.printf(">1.0um: %u\n", pmsData.particles_10um);
                    Serial.printf(">2.5um: %u\n", pmsData.particles_25um);
                    Serial.printf(">5.0um: %u\n", pmsData.particles_50um);
                    Serial.printf(">10um:  %u\n", pmsData.particles_100um);
                }
                
                if (bmeReceived) {
                    Serial.println("\nEnvironmental Data:");
                    Serial.printf("Temperature: %.2f °C\n", bmeData.temperature);
                    Serial.printf("Humidity: %.2f %%\n", bmeData.humidity);
                    Serial.printf("Pressure: %.2f hPa\n", bmeData.pressure);
                    Serial.printf("Gas Resistance: %.2f kΩ\n", bmeData.gas);
                }
                
                Serial.println("-------------------");
                xSemaphoreGive(serialMutex);
            }
        }
    }
}

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Starting PMS5003 RTOS initialization...");

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize BME680
    if (!bme.begin()) {
        Serial.println("Could not find a valid BME680 sensor!");
        while (1);
    }

    // Set up BME680 parameters
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320°C for 150 ms
    
    // Create mutex for serial communication
    serialMutex = xSemaphoreCreateMutex();
    wireMutex = xSemaphoreCreateMutex();
    
    // Create queue for sensor data
    pmsDataQueue = xQueueCreate(5, sizeof(PMS5003Data));
    bmeDataQueue = xQueueCreate(5, sizeof(BME680Data));
    
    // Initialize PMS sensor serial
    pmsSensor.begin(PMS_BAUD, SERIAL_8N1, PMS_RX, PMS_TX);
    
    // Create tasks
    xTaskCreatePinnedToCore(
        pmsReadTask,
        "PMS Read Task",
        4096,
        NULL,
        2,
        &pmsReadTaskHandle,
        1
    );

    xTaskCreatePinnedToCore(
        bmeReadTask,
        "BME Read Task",
        4096,
        NULL,
        2,
        &bmeReadTaskHandle,
        1
    );
    
    xTaskCreatePinnedToCore(
        displayTask,
        "Display Task",
        4096,
        NULL,
        1,
        &displayTaskHandle,
        0
    );
    
    Serial.println("PMS5003 RTOS initialization completed");
}

void loop() {
    // Empty loop as we're using RTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
