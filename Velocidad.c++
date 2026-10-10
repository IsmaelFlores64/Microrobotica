#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Definición de pines
const int pinA = 3;
const int PWM_salida = 6;
const int POT_sp = A0; // Pin analógico para el potenciómetro

// Inicialización de la pantalla I2C (dirección 0x27, 16 columnas, 2 filas)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variables del Encoder y tiempo
volatile int contador = 0;
unsigned long previousMillis = 0;
const long interval = 100; // Tm = 0.1 s

// Variables del PID
float pv = 0.0;
float sp = 0.0;
float cv = 0.0;
float cv1 = 0.0;
float error = 0.0;
float error1 = 0.0;
float error2 = 0.0;

// Constantes PID
float Kp = 1.0;
float Ki = 5.0;
float Kd = 0.001;
float Tm = 0.1;

void setup() {
  pinMode(pinA, INPUT);
  pinMode(PWM_salida, OUTPUT);
  Serial.begin(115200);

  // Configuración de interrupción según estándar moderno
  attachInterrupt(digitalPinToInterrupt(pinA), interrupcion, RISING);

  // Inicialización de la pantalla LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();

  // Control muestreado cada 100 ms (Tm = 0.1 s)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calculo de velocidad actual (RPM)
    pv = 10.0 * contador * (60.0 / 374.0);
    contador = 0;

    // Lectura del Set Point (0 - 380 RPM)
    sp = analogRead(POT_sp) * (380.0 / 1023.0);

    // Error actual
    error = sp - pv;

    // Ecuación de diferencias PID
    cv = cv1 + (Kp + Kd / Tm) * error + (-Kp + Ki * Tm - 2 * Kd / Tm) * error1 + (Kd / Tm) * error2;
    cv1 = cv;
    error2 = error1;
    error1 = error;

    // Saturación de la salida PID
    if (cv > 500.0) {
      cv = 500.0;
    }
    if (cv < 30.0) {
      cv = 30.0;
    }

    // Salida PWM (Mapeo de 0-500 a 0-255)
    analogWrite(PWM_salida, cv * (255.0 / 500.0));

    // Monitor Serie
    Serial.print("SP: ");
    Serial.print(sp);
    Serial.print(", PV: ");
    Serial.println(pv);

    // Actualizar pantalla LCD
    lcd.setCursor(0, 0);
    lcd.print("SP (rpm): ");
    lcd.print(sp, 1);
    lcd.print("   "); // Limpia dígitos sobrantes

    lcd.setCursor(0, 1);
    lcd.print("PV (rpm): ");
    lcd.print(pv, 1);
    lcd.print("   ");
  }
}

void interrupcion() {
  contador++;
}