    #include <Arduino.h>
    #include <SensirionI2cSps30.h>
    #include <SensirionI2CSgp40.h>
    #include <SensirionI2cScd4x.h>
    #include <Wire.h>
    #include <BLEDevice.h>  // Main library BLE
    #include <BLEServer.h>  // Library for creating Server
    #include <BLEUtils.h>   // Tools helpers
    #include <SPIFFS.h>

    // macro definitions
    // make sure that we use the proper definition of NO_ERROR
    #ifdef NO_ERROR
    #undef NO_ERROR
    #endif
    #define NO_ERROR 0
    #define SERVICE_UUID        "50106842-26c7-4e08-a41e-dda4319c2fc5"
    #define CHARACTERISTIC_UUID "2c5d2e0b-51ae-470e-8a4a-657207292a04"
    #define STATUS_UUID         "9f1d2e0b-51ae-470e-8a4a-657207292a05"
    
    // Circular buffer for offline archiving (500 last samples)
    #define MAX_BUFFER_SIZE 500
    #define ARCHIVE_PATH "/archive.log"
    
    // Circular buffer structure
    struct CircularBuffer {
        String samples[MAX_BUFFER_SIZE];
        uint32_t head = 0;     // write position
        uint32_t count = 0;    // number of stored samples (0 to MAX_BUFFER_SIZE)
        
        void add(const String &payload) {
            samples[head] = payload;
            head = (head + 1) % MAX_BUFFER_SIZE;
            if (count < MAX_BUFFER_SIZE) {
                count++;
            }
        }
        
        void flush() {
            if (count == 0) return;
            // write all samples to file
            SPIFFS.remove(ARCHIVE_PATH);
            File f = SPIFFS.open(ARCHIVE_PATH, FILE_WRITE);
            if (!f) return;
            
            uint32_t pos = (head - count + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
            for (uint32_t i = 0; i < count; i++) {
                f.println(samples[pos]);
                pos = (pos + 1) % MAX_BUFFER_SIZE;
            }
            f.close();
            Serial.println("Buffer flushed to SPIFFS");
        }
        
        void clear() {
            head = 0;
            count = 0;
        }
        
        // peek at oldest sample without removing
        String peekFront() const {
            if (count == 0) return String();
            uint32_t pos = (head + MAX_BUFFER_SIZE - count) % MAX_BUFFER_SIZE;
            return samples[pos];
        }

        // remove and return oldest sample
        String popFront() {
            if (count == 0) return String();
            uint32_t pos = (head + MAX_BUFFER_SIZE - count) % MAX_BUFFER_SIZE;
            String s = samples[pos];
            // optional: clear stored String to free memory
            samples[pos] = String();
            count--;
            return s;
        }
    };
    
    static CircularBuffer archiveBuffer;

    // Packet sequence counter (for tracking)
    static uint32_t packetSeq = 0;

    BLECharacteristic *pCharacteristic;
    BLECharacteristic *pStatusCharacteristic; // read-only status
    static bool deviceConnected = false;

    // BLE notify throttle
    static unsigned long lastNotifyTs = 0;
    const unsigned long NOTIFY_INTERVAL_MS = 100; // 100ms between notifies (~10/sec)

    // Flushing state (non-blocking flush)
    static bool flushing = false;
    static uint32_t flushPos = 0; // position index for incremental flush

    // Sensor recovery timestamps
    static unsigned long lastSuccessSps30 = 0;
    static unsigned long lastSuccessSgp40 = 0;
    static unsigned long lastSuccessScd41 = 0;
    const unsigned long SENSOR_RECOVERY_TIMEOUT = 2 * 60 * 1000UL; // 2 minutes

    // status update interval
    unsigned long lastStatusUpdate = 0;
    const unsigned long STATUS_UPDATE_INTERVAL = 10000; // 10s

    // forward declarations
    void flushArchive();
    void loadArchiveFromDisk();
    void startFlushArchive();
    void processFlushStep();

    class MyServerCallbacks : public BLEServerCallbacks {
        void onConnect(BLEServer* pServer) override {
            deviceConnected = true;
            // start non-blocking flush of archived data when a client connects
            startFlushArchive();
        }
        void onDisconnect(BLEServer* pServer) override {
            deviceConnected = false;
        }
    };

    // send via BLE characteristic if connected
    bool sendDataNow(const String &payload) {
        if (!pCharacteristic) return false;
        if (!deviceConnected) return false;
        unsigned long now = millis();
        if (now - lastNotifyTs < NOTIFY_INTERVAL_MS) {
            // too soon to notify again, caller should retry later
            return false;
        }
        // print payload to serial so we can see what is being sent over BLE
        Serial.print("Sending via BLE: ");
        Serial.println(payload);
        pCharacteristic->setValue((uint8_t*)payload.c_str(), payload.length());
        pCharacteristic->notify();
        lastNotifyTs = now;
        return true;
    }

    // Load archived data from disk into buffer on startup
    void loadArchiveFromDisk() {
        if (!SPIFFS.exists(ARCHIVE_PATH)) {
            Serial.println("No archive file to load");
            return;
        }
        File f = SPIFFS.open(ARCHIVE_PATH, FILE_READ);
        if (!f) {
            Serial.println("Failed to open archive");
            return;
        }
        uint32_t loaded = 0;
        while (f.available() && archiveBuffer.count < MAX_BUFFER_SIZE) {
            String line = f.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) {
                archiveBuffer.add(line);
                loaded++;
            }
        }
        f.close();
        Serial.print("Loaded ");
        Serial.print(loaded);
        Serial.println(" samples from disk");
    }

    // Save buffer to disk immediately (used when we cannot send right now)
    void flushArchive() {
        archiveBuffer.flush();
    }

    // Start non-blocking flush: initialize position pointer
    void startFlushArchive() {
        if (archiveBuffer.count == 0) {
            Serial.println("Nothing to flush (buffer empty)");
            return;
        }
        flushPos = (archiveBuffer.head + MAX_BUFFER_SIZE - archiveBuffer.count) % MAX_BUFFER_SIZE;
        flushing = true;
        Serial.print("Starting non-blocking flush of ");
        Serial.print(archiveBuffer.count);
        Serial.println(" samples");
    }

    // Called periodically from loop() to send one archived sample at a time
    void processFlushStep() {
        if (!flushing) return;
        if (archiveBuffer.count == 0) {
            Serial.println("Flush complete (buffer empty)");
            flushing = false;
            SPIFFS.remove(ARCHIVE_PATH);
            return;
        }
        if (!deviceConnected) {
            Serial.println("Client disconnected during flush, saving buffer to disk");
            archiveBuffer.flush();
            flushing = false;
            return;
        }

        // peek oldest sample and try to send once
        String &sample = archiveBuffer.samples[flushPos];
        if (sample.length() == 0) {
            // shouldn't happen but be robust
            archiveBuffer.popFront();
            flushPos = (flushPos + 1) % MAX_BUFFER_SIZE;
            return;
        }

        bool ok = sendDataNow(sample);
        if (ok) {
            // remove the sent sample
            archiveBuffer.popFront();
            // flushPos already points to oldest; after popFront oldest is next pos
            flushPos = (flushPos) % MAX_BUFFER_SIZE; // remain correct
            // continue next iteration (but we only do one per loop call)
        } else {
            // send failed - likely throttled; try again later
            // do not remove sample; just return and try next loop
            return;
        }
    }


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
    const unsigned long INTERVAL_SPS30 = 30000;   // 30 seconds
    const unsigned long INTERVAL_SGP40 = 30000;   // 30 seconds
    const unsigned long INTERVAL_SCD41 = 30000;   // 30 seconds

    void PrintUint64(uint64_t& value) {
    Serial.print("0x");
    Serial.print((uint32_t)(value >> 32), HEX);
    Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
}

    // --- Diagnostics and read helpers for each sensor ---
    // consolidated data structure for a full measurement
    struct AirMeasurement {
        // SPS30
        uint16_t mc1p0 = 0;
        uint16_t mc2p5 = 0;
        uint16_t mc4p0 = 0;
        uint16_t mc10p0 = 0;
        // optional number concentrations
        uint16_t nc0p5 = 0;
        uint16_t nc1p0 = 0;
        uint16_t nc2p5 = 0;
        uint16_t nc4p0 = 0;
        uint16_t nc10p0 = 0;
        uint16_t typicalParticleSize = 0;
        bool haveSps30 = false;

        // SGP40
        uint16_t srawVoc = 0;
        bool haveSgp40 = false;

        // SCD41
        uint16_t co2 = 0;
        float temp = 0.0f;
        float rh = 0.0f;
        bool haveScd41 = false;

        // timestamp for the combined payload (ms since boot)
        unsigned long ts = 0;
    };

    static AirMeasurement latestMeasurement;
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
            latestMeasurement.haveSgp40 = false;
            return;
        }
        Serial.print("SRAW_VOC: ");
        Serial.println(srawVoc);
        // store reading in latestMeasurement (no sending here)
        latestMeasurement.srawVoc = srawVoc;
        latestMeasurement.haveSgp40 = true;
        latestMeasurement.ts = millis();
        lastSuccessSgp40 = millis();
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
        // store SPS30 readings into latestMeasurement (do not send individually)
        latestMeasurement.mc1p0 = mc1p0;
        latestMeasurement.mc2p5 = mc2p5;
        latestMeasurement.mc4p0 = mc4p0;
        latestMeasurement.mc10p0 = mc10p0;
        latestMeasurement.nc0p5 = nc0p5;
        latestMeasurement.nc1p0 = nc1p0;
        latestMeasurement.nc2p5 = nc2p5;
        latestMeasurement.nc4p0 = nc4p0;
        latestMeasurement.nc10p0 = nc10p0;
        latestMeasurement.typicalParticleSize = typicalParticleSize;
        latestMeasurement.haveSps30 = true;
        latestMeasurement.ts = millis();
        lastSuccessSps30 = millis();
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
        // store reading in latestMeasurement (no sending here)
        latestMeasurement.co2 = co2;
        latestMeasurement.temp = temp;
        latestMeasurement.rh = rh;
        latestMeasurement.haveScd41 = true;
        latestMeasurement.ts = millis();
        lastSuccessScd41 = millis();
    }

    void setup() {

        Serial.begin(115200);
        while (!Serial) {
            delay(100);
        }

        // init SPIFFS for archive
        if (!SPIFFS.begin(true)) {
            Serial.println("SPIFFS Mount Failed");
        } else {
            Serial.println("SPIFFS initialized, loading archive...");
            loadArchiveFromDisk();
        }

        Wire.begin();
        sps30.begin(Wire, SPS30_I2C_ADDR_69);
        sgp40.begin(Wire);
        scd41.begin(Wire, SCD41_I2C_ADDR_62);

        // 1. Start BLE and give your device a name
        BLEDevice::init("MojCzujnikPowietrza");
        // 2. create BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
        // 3. Create a "Service" on that server
        BLEService *pService = pServer->createService(SERVICE_UUID);
        // 4. Create a "Characteristic" (data channel) inside that service
        pCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_READ |  // Ta charakterystyka jest CZYTELNA
                            BLECharacteristic::PROPERTY_NOTIFY // Można ją "subskrybować" (dostawać powiadomienia)
                            );
    // Status characteristic (read-only) - returns buffer and device state
    pStatusCharacteristic = pService->createCharacteristic(
                STATUS_UUID,
                BLECharacteristic::PROPERTY_READ
                );
    // initial status
    pStatusCharacteristic->setValue("{\"buffer\":0,\"connected\":false,\"seq\":0}");
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

        // process one archival flush step if in progress (non-blocking)
        processFlushStep();

        // periodic status update characteristic
        if (now - lastStatusUpdate >= STATUS_UPDATE_INTERVAL) {
            lastStatusUpdate = now;
            char statusBuf[128];
            int n = snprintf(statusBuf, sizeof(statusBuf), "{\"buffer\":%u,\"connected\":%s,\"seq\":%u}",
                             (unsigned)archiveBuffer.count, deviceConnected ? "true" : "false", packetSeq);
            if (n > 0) pStatusCharacteristic->setValue((uint8_t*)statusBuf, n);
        }

        // sensor recovery: if a sensor hasn't reported for SENSOR_RECOVERY_TIMEOUT, run its diag
        if ((now - lastSuccessSps30) > SENSOR_RECOVERY_TIMEOUT) {
            Serial.println("SPS30 not responding - running diagSps30()");
            diagSps30();
            lastSuccessSps30 = now; // avoid repeating too fast
        }
        if ((now - lastSuccessSgp40) > SENSOR_RECOVERY_TIMEOUT) {
            Serial.println("SGP40 not responding - running diagSgp40()");
            diagSgp40();
            lastSuccessSgp40 = now;
        }
        if ((now - lastSuccessScd41) > SENSOR_RECOVERY_TIMEOUT) {
            Serial.println("SCD41 not responding - running diagScd41()");
            diagScd41();
            lastSuccessScd41 = now;
        }

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

        // If we have fresh readings from all sensors, send one combined JSON
        if (latestMeasurement.haveSps30 && latestMeasurement.haveSgp40 && latestMeasurement.haveScd41) {
            // build improved combined JSON payload with readable names
            // seq = sequence number (for tracking), ts = timestamp(ms)
            // co2 = CO2 [ppm], temp_c = temperature [°C], humidity_rh = humidity [%]
            // voc = SRAW_VOC, pm25 = PM2.5 [µg/m³], pm10 = PM10 [µg/m³]
            unsigned long ts = latestMeasurement.ts;
            packetSeq++;  // increment sequence counter
            char payloadBuf[192];
            int len = snprintf(payloadBuf, sizeof(payloadBuf), "{\"seq\":%u,\"ts\":%lu,\"co2\":%u,\"temp_c\":%.2f,\"humidity_rh\":%.2f,\"voc\":%u,\"pm25\":%u,\"pm10\":%u}",
                               packetSeq, ts, latestMeasurement.co2, latestMeasurement.temp, latestMeasurement.rh,
                               latestMeasurement.srawVoc, latestMeasurement.mc2p5, latestMeasurement.mc10p0);

            String payload;
            if (len > 0 && len < (int)sizeof(payloadBuf)) payload = String(payloadBuf);
            else payload = String("{}");

            bool sent = false;
            if (deviceConnected) sent = sendDataNow(payload);
            if (!sent) {
                // add to circular buffer (no SPIFFS write yet)
                archiveBuffer.add(payload);
                Serial.print("Combined data archived to buffer (");
                Serial.print(archiveBuffer.count);
                Serial.println("/500 samples)");
            } else {
                Serial.println("Combined data sent via BLE");
            }

            // reset measurement flags so next cycle waits for new readings
            latestMeasurement.haveSps30 = false;
            latestMeasurement.haveSgp40 = false;
            latestMeasurement.haveScd41 = false;
        }
    }