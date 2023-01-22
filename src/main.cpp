#include <Arduino.h>
#include <Notecard.h>
#include <Motion.h>

#include <pot-hole-detection_inferencing.h>
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  4.0f  
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static int currentBufferPosition = 0;
 
#define PRODUCT_UID "com.gmail.michaelmalinowski777:pothole_detector"
#define NOTEHUB_LOCATION_FILE "pothole-location.qo"

Notecard notecard;

Motion motion;
float accelModifiedData[3];

void startNoteCardSync() {
    J *req = NoteNewRequest("hub.sync");
    NoteRequest(req);
}

bool isNoteCardSynced() {
    J *req = NoteNewRequest("hub.sync.status");
    JAddBoolToObject(req, "sync", true);

    J *res = NoteRequestResponse(req);
    char *status = JGetString(res, "status");
    Serial.println(status);

    //Notecard status check for completion
    if (status == "connected {connected-closed}") {
        return true;
    }

    return false;
}

void setNoteCardProductId(int maxRetry = 5) {
    bool success = NoteSetProductID(PRODUCT_UID);
    if (!success) {
        Serial.print("Failed to set NoteCard Product ID:");
        Serial.println(PRODUCT_UID);
        if (maxRetry > 0) {
            --maxRetry;
            delay(1000);
            setNoteCardProductId(maxRetry);
        }
        return;
    }
}

void setNoteCardLocationMode(int maxRetry = 5) {
    bool success = NoteSetLocationMode("continuous", 0);
    if (!success) {
        Serial.print("Failed to set NoteCard Location Mode");
        if (maxRetry > 0) {
            --maxRetry;
            delay(1000);
            setNoteCardProductId(maxRetry);
        }
        return;
    }
}

void addLocationNote() {
    J *req = NoteNewRequest("card.location");
    J *res = NoteRequestResponse(req);
    char *status = JGetString(res, "status");
    float lat = JGetNumber(res, "lat");
    float lon = JGetNumber(res, "lon");
    if (strstr(status, "GPS updated")) {
        J *note = JCreateObject();
        JAddNumberToObject(note, "lat", lat);
        JAddNumberToObject(note, "lon", lon);

        NoteAdd(NOTEHUB_LOCATION_FILE, note, false);
    }
}

bool hasHitPothole() {
    //Clears the buffer
    if (currentBufferPosition == 0) {
        for (int i=0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ++i) {
            buffer[i] = 0;
        }
    }

    //Loads the buffer with acceleration data
    buffer[currentBufferPosition] = accelModifiedData[0] * CONVERT_G_TO_MS2;
    buffer[currentBufferPosition + 1] = accelModifiedData[1] * CONVERT_G_TO_MS2;
    buffer[currentBufferPosition + 2] = accelModifiedData[2] * CONVERT_G_TO_MS2;

    


    currentBufferPosition += 3;
    if (currentBufferPosition < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        return false;
    }
    currentBufferPosition = 0;

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return false;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return false;
    }

    // print the predictions (FOR DEBUG)
    // ei_printf("Predictions ");
    // ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
    //     result.timing.dsp, result.timing.classification, result.timing.anomaly);
    // ei_printf(": \n");
    // for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    //     ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    // }
    // #if EI_CLASSIFIER_HAS_ANOMALY == 1
    //     ei_printf("    anomaly score: %.3f\n", result.anomaly);
    // #endif

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].label == "pothole" && result.classification[ix].value > 0.90) {
            return true;
        }
    }
    return false;
}

void setup() {
    delay(2500);
    Serial.begin(115200);
    
    /*NoteCard Debugging Option*/
    notecard.begin();
    notecard.setDebugOutputStream(Serial);
    /*#########################*/
    setNoteCardProductId();
    startNoteCardSync();
    setNoteCardLocationMode();

    //Start MPU
    motion.start(0);
    motion.setAccelDrift(-50,0,400);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    motion.updateMotionData();
    motion.getAccelDataInGravities(accelModifiedData);

    //Checks if the mpu is relatively motionless to start notehub sync
    if (motion.hasStablized(10, 300)) {
        startNoteCardSync();
    }
    //Checks if pothole has been hit
    digitalWrite(LED_BUILTIN, LOW);
    if (hasHitPothole()) {
        addLocationNote();
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
    }
    delayMicroseconds(16.6 * 1000);
}