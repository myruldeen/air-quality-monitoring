#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SensirionI2CScd4x.h>

#define MQTT_MAX_PACKET_SIZE 1024

// WiFi settings
const char* ssid = "norazlin@unifi";
const char* password = "bkh223811286";

// MQTT settings
const char* mqtt_server = "denodev.duckdns.org";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_topic = "sensors/environmental";
const char* device_id = "env_sensor_01";

// Add LED pin for status indication
#define LED_PIN 2

// Create WiFi and MQTT clients
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Pin definitions
#define PMS_RX 16
#define PMS_TX 17
#define PMS_BAUD 9600

#define I2C_SDA 21 
#define I2C_SCL 22

// Task handles
TaskHandle_t pmsReadTaskHandle = NULL;
TaskHandle_t bmeReadTaskHandle = NULL;
TaskHandle_t scdReadTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t mqttTaskHandle = NULL;

// Queue handle for sensor data
QueueHandle_t pmsDataQueue;
QueueHandle_t bmeDataQueue;
QueueHandle_t scdDataQueue;
QueueHandle_t mqttQueue;

// Mutex for serial communication
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t wireMutex;

// Hardware Serial for PMS sensor
HardwareSerial pmsSensor(2);
Adafruit_BME680 bme;
SensirionI2CScd4x scd4x;

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

// Structure for SCD41 data
struct SCD41Data {
    uint16_t co2;
    float temperature;
    float humidity;
    bool isValid;
};

// Structure to hold combined sensor data
struct CombinedData {
    PMS5003Data pms;
    BME680Data bme;
    SCD41Data scd;
    bool pmsValid;
    bool bmeValid;
    bool scdValid;
    unsigned long timestamp;
};

void setupWiFi() {
    WiFi.mode(WIFI_STA);  // Set WiFi to station mode
    WiFi.disconnect();     // Disconnect from any previous connections
    delay(100);
    
    Serial.printf("Connecting to %s", ssid);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed!");
        ESP.restart();  // Restart ESP32 if WiFi connection fails
    }
}

void reconnectMQTT() {
    int attempts = 0;
    while (!mqttClient.connected() && attempts < 3) {  // Limit retry attempts
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32Env-" + String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("connected");
            digitalWrite(LED_PIN, HIGH);  // Turn on LED when connected
            delay(100);
            digitalWrite(LED_PIN, LOW);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retrying in 5 seconds");
            attempts++;
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}


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

void mqttTask(void* parameter) {
    CombinedData sensorData;
    
    while (1) {
        // Check WiFi connection first
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected. Reconnecting...");
            setupWiFi();
        }
        
        if (xQueueReceive(mqttQueue, &sensorData, portMAX_DELAY) == pdTRUE) {
            if (!mqttClient.connected()) {
                reconnectMQTT();
            }
            
            if ((sensorData.pmsValid || sensorData.bmeValid || sensorData.scdValid) && mqttClient.connected()) {
                DynamicJsonDocument doc(1024);  // Changed from StaticJsonDocument
                
                doc["device_id"] = device_id;
                doc["timestamp"] = sensorData.timestamp;
                
                if (sensorData.pmsValid) {
                    JsonObject pmsData = doc.createNestedObject("pms5003");
                    pmsData["pm1_0"] = sensorData.pms.pm10_standard;
                    pmsData["pm2_5"] = sensorData.pms.pm25_standard;
                    pmsData["pm10_0"] = sensorData.pms.pm100_standard;
                    pmsData["particles_0_3"] = sensorData.pms.particles_03um;
                    pmsData["particles_0_5"] = sensorData.pms.particles_05um;
                    pmsData["particles_1_0"] = sensorData.pms.particles_10um;
                    pmsData["particles_2_5"] = sensorData.pms.particles_25um;
                    pmsData["particles_5_0"] = sensorData.pms.particles_50um;
                    pmsData["particles_10_0"] = sensorData.pms.particles_100um;
                }
                
                if (sensorData.bmeValid) {
                    JsonObject bmeData = doc.createNestedObject("bme680");
                    bmeData["temperature"] = round(sensorData.bme.temperature * 100.0) / 100.0;
                    bmeData["humidity"] = round(sensorData.bme.humidity * 100.0) / 100.0;
                    bmeData["pressure"] = round(sensorData.bme.pressure * 100.0) / 100.0;
                    bmeData["gas"] = round(sensorData.bme.gas * 100.0) / 100.0;
                }
                
                if (sensorData.scdValid) {
                    JsonObject scdData = doc.createNestedObject("scd41");
                    scdData["co2"] = sensorData.scd.co2;
                    scdData["temperature"] = round(sensorData.scd.temperature * 100.0) / 100.0;
                    scdData["humidity"] = round(sensorData.scd.humidity * 100.0) / 100.0;
                }
                
                String jsonString;
                serializeJson(doc, jsonString);
                
                if (mqttClient.publish(mqtt_topic, jsonString.c_str(), false)) {
                    digitalWrite(LED_PIN, HIGH);
                    delay(100);
                    digitalWrite(LED_PIN, LOW);
                    Serial.println("Published to MQTT:");
                    Serial.println(jsonString);
                } else {
                    Serial.println("Failed to publish to MQTT!");
                    Serial.printf("MQTT state: %d\n", mqttClient.state());
                }
            }
        }
        mqttClient.loop();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
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

// Task to read SCD41 sensor
void scdReadTask(void *parameter) {
    SCD41Data sensorData;
    static unsigned long failedReads = 0;
    
    while (1) {
        sensorData.isValid = false;
        
        if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            uint16_t co2;
            float temperature;
            float humidity;
            uint16_t error;
            
            error = scd4x.readMeasurement(co2, temperature, humidity);
            
            if (error == 0) {
                sensorData.co2 = co2;
                sensorData.temperature = temperature;
                sensorData.humidity = humidity;
                sensorData.isValid = true;
                failedReads = 0;
                
                if (xQueueSend(scdDataQueue, &sensorData, 0) != pdTRUE) {
                    xQueueReset(scdDataQueue);
                    xQueueSend(scdDataQueue, &sensorData, 0);
                }
            } else {
                failedReads++;
                if (failedReads > 5) {
                    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        Serial.println("SCD41 read failed multiple times!");
                        xSemaphoreGive(serialMutex);
                    }
                    // Try to reinitialize
                    scd4x.stopPeriodicMeasurement();
                    delay(500);
                    scd4x.startPeriodicMeasurement();
                    failedReads = 0;
                }
            }
            xSemaphoreGive(wireMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // SCD41 needs about 5 seconds between measurements
    }
}

// Task to display sensor data
void displayTask(void *parameter) {
    CombinedData combinedData;
    static unsigned long readCount = 0;
    const TickType_t xMaxWait = pdMS_TO_TICKS(5000);  // Increased timeout for SCD41
    
    while (1) {
        // Clear previous state
        combinedData.pmsValid = false;
        combinedData.bmeValid = false;
        combinedData.scdValid = false;
        
        // Try to get all sensor data
        bool haveAllData = false;
        
        // Wait for all sensors to have data
        if (xQueueReceive(pmsDataQueue, &combinedData.pms, xMaxWait) == pdTRUE &&
            xQueueReceive(bmeDataQueue, &combinedData.bme, xMaxWait) == pdTRUE &&
            xQueueReceive(scdDataQueue, &combinedData.scd, xMaxWait) == pdTRUE) {
            
            combinedData.pmsValid = true;
            combinedData.bmeValid = true;
            combinedData.scdValid = true;
            combinedData.timestamp = millis();
            haveAllData = true;
        }
        
        // Only display if we have data from all sensors
        if (haveAllData) {
            
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
                
                Serial.println("\nBME680 Environmental Data:");
                Serial.printf("Temperature: %.2f °C\n", combinedData.bme.temperature);
                Serial.printf("Humidity: %.2f %%\n", combinedData.bme.humidity);
                Serial.printf("Pressure: %.2f hPa\n", combinedData.bme.pressure);
                Serial.printf("Gas Resistance: %.2f kΩ\n", combinedData.bme.gas);
                
                Serial.println("\nSCD41 CO2 Data:");
                Serial.printf("CO2: %u ppm\n", combinedData.scd.co2);
                Serial.printf("Temperature: %.2f °C\n", combinedData.scd.temperature);
                Serial.printf("Humidity: %.2f %%\n", combinedData.scd.humidity);
                
                Serial.println("-------------------");

                xQueueSend(mqttQueue, &combinedData, 0);
                xSemaphoreGive(serialMutex);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    mqttClient.setBufferSize(1024);
    setupWiFi();
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setKeepAlive(60);
    
    Serial.println("Starting sensor initialization...");
    
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Initialize BME680
    if (!bme.begin()) {
        Serial.println("Could not find BME680 sensor!");
    } else {
        Serial.println("BME680 initialized");
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);
    }
    
    // Initialize SCD41
    scd4x.begin(Wire);
    
    // Stop potentially previously started measurement
    scd4x.stopPeriodicMeasurement();
    delay(500);
    
    // Start periodic measurement
    uint16_t error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.println("Error starting SCD41 measurement!");
    } else {
        Serial.println("SCD41 initialized");
    }
    
    // Create mutexes and queues
    serialMutex = xSemaphoreCreateMutex();
    wireMutex = xSemaphoreCreateMutex();
    
    pmsDataQueue = xQueueCreate(5, sizeof(PMS5003Data));
    bmeDataQueue = xQueueCreate(5, sizeof(BME680Data));
    scdDataQueue = xQueueCreate(5, sizeof(SCD41Data));
    mqttQueue = xQueueCreate(5, sizeof(CombinedData));
    
    // Initialize PMS sensor
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
        scdReadTask,
        "SCD Read Task",
        4096,
        NULL,
        2,
        &scdReadTaskHandle,
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

    xTaskCreatePinnedToCore(
        mqttTask,
        "MQTT Task",
        8192, 
        NULL,
        1,
        &mqttTaskHandle,
        1
    );
    
    Serial.println("Sensor initialization completed");
}

void loop() {
    // Empty loop as we're using RTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
