void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 готова. Введите число:");
}

void loop() {
    if (Serial.available()) {
        int number = Serial.parseInt();  // Читаем число из порта
        Serial.print("Получено число: ");
        Serial.println(number);
    }
}
