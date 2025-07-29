
void setup () {
    Serial.begin(115200);
}

void loop () {
    if(Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        String response = processComand(cmd);

        Serial.println(response);
    }
}

String processComand(String cmd) {
    if(cmd == "WHITE")
        return "BLACK";
    
    if(cmd == "BLACK")
        return "WHITE";
}