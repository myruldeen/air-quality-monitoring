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

struct BME680Data {
    float temperature;
    float humidity;
    float pressure;
    float gas;
    bool isValid;
};

// Structure to hold combined sensor data
struct CombinedData {
    PMS5003Data pms;
    BME680Data bme;
    bool pmsValid;
    bool bmeValid;
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
    static unsigned long lastSuccessfulRead = 0;
    static unsigned long failedReads = 0;
    
    while (1) {
        sensorData.isValid = false;
        
        if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (bme.performReading()) {
                sensorData.temperature = bme.temperature;
                sensorData.humidity = bme.humidity;
                sensorData.pressure = bme.pressure / 100.0;
                sensorData.gas = bme.gas_resistance / 1000.0;
                sensorData.isValid = true;
                lastSuccessfulRead = millis();
                
                if (failedReads > 0) {
                    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        Serial.println("BME680 recovered after failures");
                        xSemaphoreGive(serialMutex);
                    }
                }
                failedReads = 0;
                
                if (xQueueSend(bmeDataQueue, &sensorData, 0) != pdTRUE) {
                    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        Serial.println("BME680 queue full, resetting");
                        xSemaphoreGive(serialMutex);
                    }
                    xQueueReset(bmeDataQueue);
                    xQueueSend(bmeDataQueue, &sensorData, 0);
                }
            } else {
                failedReads++;
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    Serial.printf("BME680 read failed. Attempt #%lu\n", failedReads);
                    xSemaphoreGive(serialMutex);
                }
                
                if (failedReads > 5) {
                    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        Serial.println("BME680 read failed multiple times!");
                        Serial.println("Attempting to reinitialize BME680...");
                        xSemaphoreGive(serialMutex);
                    }
                    
                    if (bme.begin()) {
                        bme.setTemperatureOversampling(BME680_OS_8X);
                        bme.setHumidityOversampling(BME680_OS_2X);
                        bme.setPressureOversampling(BME680_OS_4X);
                        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
                        bme.setGasHeater(320, 150);
                    }
                    failedReads = 0;
                }
            }
            xSemaphoreGive(wireMutex);
        } else {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                Serial.println("Failed to take wire mutex for BME680");
                xSemaphoreGive(serialMutex);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Task to display sensor data
void displayTask(void *parameter) {
    CombinedData combinedData;
    static unsigned long readCount = 0;
    const TickType_t xMaxWait = pdMS_TO_TICKS(2000);  // 2 second timeout
    
    while (1) {
        // Clear previous state
        combinedData.pmsValid = false;
        combinedData.bmeValid = false;
        
        // Try to get both sensor data
        bool haveBothData = false;
        
        // Wait for both sensors to have data
        if (xQueueReceive(pmsDataQueue, &combinedData.pms, xMaxWait) == pdTRUE) {
            combinedData.pmsValid = true;
            
            // If we got PMS data, try to get BME data
            if (xQueueReceive(bmeDataQueue, &combinedData.bme, xMaxWait) == pdTRUE) {
                combinedData.bmeValid = true;
                haveBothData = true;
            } else {
                // If we didn't get BME data, skip this cycle
                continue;
            }
        }
        
        // Only display if we have data from both sensors
        if (haveBothData) {
            readCount++;
            
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                Serial.printf("\nRead #%lu\n", readCount);
                
                Serial.println("\nPM Concentrations (Standard):");
                Serial.printf("PM1.0: %u μg/m3\n", combinedData.pms.pm10_standard);
                Serial.printf("PM2.5: %u μg/m3\n", combinedData.pms.pm25_standard);
                Serial.printf("PM10:  %u μg/m3\n", combinedData.pms.pm100_standard);
                
                Serial.println("\nParticle counts per 0.1L:");
                Serial.printf(">0.3um: %u\n", combinedData.pms.particles_03um);
                Serial.printf(">0.5um: %u\n", combinedData.pms.particles_05um);
                Serial.printf(">1.0um: %u\n", combinedData.pms.particles_10um);
                Serial.printf(">2.5um: %u\n", combinedData.pms.particles_25um);
                Serial.printf(">5.0um: %u\n", combinedData.pms.particles_50um);
                Serial.printf(">10um:  %u\n", combinedData.pms.particles_100um);
                
                Serial.println("\nEnvironmental Data:");
                Serial.printf("Temperature: %.2f °C\n", combinedData.bme.temperature);
                Serial.printf("Humidity: %.2f %%\n", combinedData.bme.humidity);
                Serial.printf("Pressure: %.2f hPa\n", combinedData.bme.pressure);
                Serial.printf("Gas Resistance: %.2f kΩ\n", combinedData.bme.gas);
                
                Serial.println("-------------------");
                xSemaphoreGive(serialMutex);
            }
        }
        
        // Small delay before next iteration
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Starting PMS5003 RTOS initialization...");

    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100); // Give I2C time to stabilize
    
    Serial.println("Attempting to initialize BME680...");
    
    // Try both possible I2C addresses
    if (!bme.begin(0x76)) {
        Serial.println("Failed with address 0x76, trying 0x77...");
        if (!bme.begin(0x77)) {
            Serial.println("Could not find a valid BME680 sensor on either address!");
            Serial.println("Check your wiring and I2C address!");
        } else {
            Serial.println("BME680 found at address 0x77");
        }
    } else {
        Serial.println("BME680 found at address 0x76");
    }
    
    // Verify I2C connection
    Wire.beginTransmission(0x76);
    byte error = Wire.endTransmission();
    Serial.printf("I2C Error status for 0x76: %d\n", error);
    // 0 = success
    // 2 = received NACK on transmit of address
    // 3 = received NACK on transmit of data
    // 4 = other error
    
    Wire.beginTransmission(0x77);
    error = Wire.endTransmission();
    Serial.printf("I2C Error status for 0x77: %d\n", error);
    
    if (bme.begin()) {
        Serial.println("BME680 initialized successfully");
        Serial.println("Configuring BME680 settings...");
        
        // Configure with error checking
        bool success = true;
        success &= bme.setTemperatureOversampling(BME680_OS_8X);
        success &= bme.setHumidityOversampling(BME680_OS_2X);
        success &= bme.setPressureOversampling(BME680_OS_4X);
        success &= bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        success &= bme.setGasHeater(320, 150);
        
        if (success) {
            Serial.println("BME680 configuration successful");
        } else {
            Serial.println("Failed to configure some BME680 settings!");
        }
    }
    
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
