#define PWM_PIN 11  // Pino de saída PWM (OC2A)

void setup() {
    Serial.begin(115200);  // Inicia comunicação serial

    pinMode(PWM_PIN, OUTPUT);

    // Configuração do Timer 2 para Fast PWM com ~31.37 kHz no pino 11 (OC2A)
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2A1); // Fast PWM, saída não-invertida
    TCCR2B = (1 << WGM22) | (1 << CS20);  // Prescaler = 1, modo Fast PWM (TOP = OCR2A)

    OCR2A = 255;  // Definir o TOP do PWM
    OCR2B = 0;    // Começa com PWM em 0%
}

void loop() {
    if (Serial.available()) {
        int pwm_value = Serial.parseInt(); // Lê valor da Serial
        if (pwm_value >= 0 && pwm_value <= 255) {
            OCR2B = pwm_value;  // Aplica o PWM no pino 11
        }
        Serial.println(OCR2B);
    }
}
