    #include <Arduino.h>
    #include <SensirionI2cSps30.h>
    #include <SensirionI2CSgp40.h>
    #include <SensirionI2cScd4x.h>
    #include <Wire.h>
    #include <BLEDevice.h>  // Main library BLE
    #include <BLEServer.h>  // Library for creating Server
    #include <BLEUtils.h>   // Tools helpers

    // macro definitions
    // make sure that we use the proper definition of NO_ERROR
    #ifdef NO_ERROR
    #undef NO_ERROR
    #endif
    #define NO_ERROR 0
    #define SERVICE_UUID        "50106842-26c7-4e08-a41e-dda4319c2fc5"
    #define CHARACTERISTIC_UUID "2c5d2e0b-51ae-470e-8a4a-657207292a04"
    
    BLECharacteristic *pCharacteristic;

    SensirionI2cSps30 sps30;
    SensirionI2CSgp40 sgp40;
    SensirionI2cScd4x scd41;

    static char errorMessage_sps30[64];
    static int16_t error_sps30;

    static char errorMessage_scd41[64];
    static int16_t error_scd41;

    // --- Timing variables for non-blocking sensor reads ---
    unsigned long lastReadSps30 = 0;
    unsigned long lastReadSgp40 = 0;
    unsigned long lastReadScd41 = 0;
    const unsigned long INTERVAL_SPS30 = 10000;   // 10 second
    const unsigned long INTERVAL_SGP40 = 10000;   // 10 second
    const unsigned long INTERVAL_SCD41 = 10000;   // 10 seconds

    void PrintUint64(uint64_t& value) {
    Serial.print("0x");
    Serial.print((uint32_t)(value >> 32), HEX);
    Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
}

    // --- Diagnostics and read helpers for each sensor ---
    void diagSgp40() {
        uint16_t error_sgp40 = 0;
        char errorMessage_sgp40[128];
        uint16_t serialNumber_sgp40[3] = {0};

        error_sgp40 = sgp40.getSerialNumber(serialNumber_sgp40, 3);
        if (error_sgp40) {
            Serial.print("SGP40 getSerialNumber error: ");
            errorToString(error_sgp40, errorMessage_sgp40, sizeof errorMessage_sgp40);
            Serial.println(errorMessage_sgp40);
        } else {
            Serial.print("SGP40 SerialNumber: 0x");
            for (size_t i = 0; i < 3; i++) {
                uint16_t value = serialNumber_sgp40[i];
                if (value < 4096) Serial.print("0");
                if (value < 256) Serial.print("0");
                if (value < 16) Serial.print("0");
                Serial.print(value, HEX);
            }
            Serial.println();
        }

        uint16_t testResult_sgp40 = 0;
        error_sgp40 = sgp40.executeSelfTest(testResult_sgp40);
        if (error_sgp40) {
            Serial.print("SGP40 executeSelfTest error: ");
            errorToString(error_sgp40, errorMessage_sgp40, sizeof errorMessage_sgp40);
            Serial.println(errorMessage_sgp40);
        } else if (testResult_sgp40 != 0xD400) {
            Serial.print("SGP40 self-test failed, result: 0x");
            Serial.println(testResult_sgp40, HEX);
        }
    }

    void readSgp40() {
        uint16_t error_sgp40 = 0;
        char errorMessage_sgp40[128];
        const uint16_t defaultRh = 0x8000; // disable humidity compensation
        const uint16_t defaultT  = 0x6666; // disable temperature compensation
        uint16_t srawVoc = 0;

        error_sgp40 = sgp40.measureRawSignal(defaultRh, defaultT, srawVoc);
        if (error_sgp40) {
            Serial.print("SGP40 measureRawSignal error: ");
            errorToString(error_sgp40, errorMessage_sgp40, sizeof errorMessage_sgp40);
            Serial.println(errorMessage_sgp40);
        } else {
            Serial.print("SRAW_VOC: ");
            Serial.println(srawVoc);
        }
    }

    void diagSps30() {
        // Use existing global sps30 object to read metadata and start measurement
        sps30.stopMeasurement();
        int8_t serialNumber_sps30[32] = {0};
        int8_t productType_sps30[8] = {0};
        sps30.readSerialNumber(serialNumber_sps30, 32);
        Serial.print("SPS30 serialNumber: ");
        Serial.println((const char*)serialNumber_sps30);
        sps30.readProductType(productType_sps30, 8);
        Serial.print("SPS30 productType: ");
        Serial.println((const char*)productType_sps30);
        sps30.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_UINT16);
        delay(100);
    }

    void readSps30() {
        uint16_t dataReadyFlag = 0;
        uint16_t mc1p0 = 0, mc2p5 = 0, mc4p0 = 0, mc10p0 = 0;
        uint16_t nc0p5 = 0, nc1p0 = 0, nc2p5 = 0, nc4p0 = 0, nc10p0 = 0;
        uint16_t typicalParticleSize = 0;
        char errorMessage[64];
        int16_t error = sps30.readDataReadyFlag(dataReadyFlag);
        if (error != NO_ERROR) {
            Serial.print("SPS30 readDataReadyFlag error: ");
            errorToString(error, errorMessage, sizeof errorMessage);
            Serial.println(errorMessage);
            return;
        }
        Serial.print("SPS30 dataReadyFlag: "); Serial.println(dataReadyFlag);

        error = sps30.readMeasurementValuesUint16(mc1p0, mc2p5, mc4p0, mc10p0,
                                                   nc0p5, nc1p0, nc2p5, nc4p0,
                                                   nc10p0, typicalParticleSize);
        if (error != NO_ERROR) {
            Serial.print("SPS30 readMeasurementValuesUint16 error: ");
            errorToString(error, errorMessage, sizeof errorMessage);
            Serial.println(errorMessage);
            return;
        }

        Serial.print("mc1p0: "); Serial.print(mc1p0); Serial.print("\t");
        Serial.print("mc2p5: "); Serial.print(mc2p5); Serial.print("\t");
        Serial.print("mc4p0: "); Serial.print(mc4p0); Serial.print("\t");
        Serial.print("mc10p0: "); Serial.print(mc10p0); Serial.print("\t");
        Serial.print("nc0p5: "); Serial.print(nc0p5); Serial.print("\t");
        Serial.print("nc1p0: "); Serial.print(nc1p0); Serial.print("\t");
        Serial.print("nc2p5: "); Serial.print(nc2p5); Serial.print("\t");
        Serial.print("nc4p0: "); Serial.print(nc4p0); Serial.print("\t");
        Serial.print("nc10p0: "); Serial.print(nc10p0); Serial.print("\t");
        Serial.print("typicalParticleSize: "); Serial.print(typicalParticleSize);
        Serial.println();
    }

    void diagScd41() {
        uint16_t error_scd41 = 0;
        char errorMessage_scd41[128];
        uint64_t serialNumber_scd41 = 0;

        delay(30);
        // Ensure sensor is in clean state
        error_scd41 = scd41.wakeUp();
        if (error_scd41 != NO_ERROR) {
            Serial.print("SCD41 wakeUp error: ");
            errorToString(error_scd41, errorMessage_scd41, sizeof errorMessage_scd41);
            Serial.println(errorMessage_scd41);
        }
        error_scd41 = scd41.stopPeriodicMeasurement();
        if (error_scd41 != NO_ERROR) {
            Serial.print("SCD41 stopPeriodicMeasurement error: ");
            errorToString(error_scd41, errorMessage_scd41, sizeof errorMessage_scd41);
            Serial.println(errorMessage_scd41);
        }
        error_scd41 = scd41.reinit();
        if (error_scd41 != NO_ERROR) {
            Serial.print("SCD41 reinit error: ");
            errorToString(error_scd41, errorMessage_scd41, sizeof errorMessage_scd41);
            Serial.println(errorMessage_scd41);
        }

        error_scd41 = scd41.getSerialNumber(serialNumber_scd41);
        if (error_scd41 != NO_ERROR) {
            Serial.print("SCD41 getSerialNumber error: ");
            errorToString(error_scd41, errorMessage_scd41, sizeof errorMessage_scd41);
            Serial.println(errorMessage_scd41);
            return;
        }
        Serial.print("SCD41 serial number: "); PrintUint64(serialNumber_scd41); Serial.println();

        error_scd41 = scd41.startPeriodicMeasurement();
        if (error_scd41 != NO_ERROR) {
            Serial.print("SCD41 startPeriodicMeasurement error: ");
            errorToString(error_scd41, errorMessage_scd41, sizeof errorMessage_scd41);
            Serial.println(errorMessage_scd41);
            return;
        }
    }

    void readScd41() {
        uint16_t error_scd41 = 0;
        char errorMessage_scd41[128];
        bool dataReady = false;
        uint16_t co2 = 0;
        float temp = 0.0f;
        float rh = 0.0f;

        // Check data ready status (non-blocking)
        error_scd41 = scd41.getDataReadyStatus(dataReady);
        if (error_scd41 != NO_ERROR) {
            Serial.print("SCD41 getDataReadyStatus error: ");
            errorToString(error_scd41, errorMessage_scd41, sizeof errorMessage_scd41);
            Serial.println(errorMessage_scd41);
            return;
        }

        // Only read if data is ready
        if (!dataReady) {
            Serial.println("SCD41 data not ready, skipping read");
            return;
        }

        error_scd41 = scd41.readMeasurement(co2, temp, rh);
        if (error_scd41 != NO_ERROR) {
            Serial.print("SCD41 readMeasurement error: ");
            errorToString(error_scd41, errorMessage_scd41, sizeof errorMessage_scd41);
            Serial.println(errorMessage_scd41);
            return;
        }

        Serial.print("CO2 concentration [ppm]: "); Serial.println(co2);
        Serial.print("Temperature [°C]: "); Serial.println(temp);
        Serial.print("Relative Humidity [RH]: "); Serial.println(rh);
    }

    void setup() {

        Serial.begin(115200);
        while (!Serial) {
            delay(100);
        }

        Wire.begin();
        sps30.begin(Wire, SPS30_I2C_ADDR_69);
        sgp40.begin(Wire);
        scd41.begin(Wire, SCD41_I2C_ADDR_62);

        // 1. Start BLE and give your device a name
        BLEDevice::init("MojCzujnikPowietrza");
        // 2. create BLE Server
        BLEServer *pServer = BLEDevice::createServer();
        // 3. Create a "Service" on that server
        BLEService *pService = pServer->createService(SERVICE_UUID);
        // 4. Create a "Characteristic" (data channel) inside that service
        pCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_READ |  // Ta charakterystyka jest CZYTELNA
                            BLECharacteristic::PROPERTY_NOTIFY // Można ją "subskrybować" (dostawać powiadomienia)
                            );
        // 5. Start the service
        pService->start();
        // 6. Start "Advertising"
        BLEAdvertising *pAdvertising = pServer->getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->start();


        // Run diagnostics / startup checks for each sensor
        diagSgp40();
        diagSps30();
        diagScd41();
    }


    void loop() {
        unsigned long now = millis();

        // Read SPS30 every INTERVAL_SPS30 ms
        if (now - lastReadSps30 >= INTERVAL_SPS30) {
            lastReadSps30 = now;
            readSps30();
        }

        // Read SGP40 every INTERVAL_SGP40 ms
        if (now - lastReadSgp40 >= INTERVAL_SGP40) {
            lastReadSgp40 = now;
            readSgp40();
        }

        // Read SCD41 every INTERVAL_SCD41 ms
        if (now - lastReadScd41 >= INTERVAL_SCD41) {
            lastReadScd41 = now;
            readScd41();
        }
    }