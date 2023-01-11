#include <Arduino.h>
#include <Notecard.h>

#define PRODUCT_UID "com.gmail.michaelmalinowski777:pothole_detector"
#define NOTEHUB_LOCATION_FILE "pothole-location.qo"

Notecard notecard;

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
    double* lat;
    double* lon;
    JTIME *time;
    char* statusBuf;
    int statusLen;
    bool success = NoteGetLocation(lat, lon, time, statusBuf, statusLen);
    if (!success) {
        Serial.println("Failed to get location of note card");
        return;
    }

    Serial.println(*lat);
    Serial.println(*lon);
    
    J *note;
    JAddNumberToObject(note, "lat", *lat);
    JAddNumberToObject(note, "lon", *lon);
    //JAddNumberToObject(req, "time", );

    NoteAdd(NOTEHUB_LOCATION_FILE, note, true);
}

void setup() {
    delay(2500);
    Serial.begin(115200);
    
    //NoteCard Debugging Option###########
    notecard.begin();
    notecard.setDebugOutputStream(Serial);
    //####################################

    setNoteCardProductId();
    setNoteCardLocationMode();
}

void loop() {
    //addLocationNote();
    delay(30000);
}