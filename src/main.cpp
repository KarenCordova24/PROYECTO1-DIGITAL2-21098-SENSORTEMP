
//KAREN LEONOR CÓRDOVA LÓPEZ

#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_adc_cal.h"

#define SNLM35 35
#define BTN_TEMP 13 
#define RED 25
#define GREEN 2
#define BLUE 15
#define servoPin 32 

int segmentPins[7] = {18, 19, 21, 22, 23, 12, 26};
int commonPins[3] = {27, 4, 5};
int Dot = 14;

float TempC_LM35 = 0.0;
int temp = 0;
int placeValuesofTemp[4];

int tempRefresh = 1000; // refrescar la temperatura cada 1 segundo
int sevSegRefresh = 5;

unsigned long time_now = 0;
bool buttonPressed = false;
bool temperatureTaken = false;
unsigned long lastButtonPressTime = 0;
unsigned long debounceDelay = 50;
bool displaysOn = false;
int lastButtonState = HIGH;

uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void setup() {
  analogReadResolution(12);

  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
  }

  for (int i = 0; i < 3; i++) {
    pinMode(commonPins[i], OUTPUT);
  }

  pinMode(Dot, OUTPUT);
  pinMode(BTN_TEMP, INPUT_PULLUP);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // Configuración de los temporizadores del LEDC
  ledcSetup(LEDC_CHANNEL_0, 5000, 13);
  ledcSetup(LEDC_CHANNEL_1, 5000, 13);
  ledcSetup(LEDC_CHANNEL_2, 5000, 13);
  ledcAttachPin(RED, LEDC_CHANNEL_0);
  ledcAttachPin(GREEN, LEDC_CHANNEL_1);
  ledcAttachPin(BLUE, LEDC_CHANNEL_2);

  // Configuración del módulo LEDC para el servo
  ledcSetup(3, 50, 10); // Canal 3 para el servo, frecuencia 50 Hz, resolución de 10 bits
  ledcAttachPin(servoPin, 3); // Asignar el pin del servo al canal 3

  Serial.begin(115200);
}

void loop() {
  int reading = digitalRead(BTN_TEMP);
  unsigned long currentTime = millis();

  if (!displaysOn && reading == LOW) {
    displaysOn = true;
  }

  if (displaysOn) {
    if (reading != lastButtonState) {
      lastButtonState = reading;

      if (reading == LOW && (currentTime - lastButtonPressTime) > debounceDelay) {
        lastButtonPressTime = currentTime;

        // Actualizar la temperatura al presionar el botón nuevamente
        temperatureTaken = false;
      }
    }
  }

  if (temperatureTaken) {
    // Mostrar la temperatura si ha sido tomada y el botón se ha presionado
    int number[10][7] = {
      {1, 1, 1, 1, 1, 1, 0}, // 0
      {0, 0, 0, 0, 1, 1, 0}, // 1
      {1, 1, 0, 1, 1, 0, 1}, // 2
      {1, 1, 1, 1, 0, 0, 1}, // 3
      {0, 1, 1, 0, 0, 1, 1}, // 4
      {1, 0, 1, 1, 0, 1, 1}, // 5
      {1, 0, 1, 1, 1, 1, 1}, // 6
      {1, 1, 1, 0, 0, 0, 0}, // 7
      {1, 1, 1, 1, 1, 1, 1}, // 8
      {1, 1, 1, 0, 0, 1, 1}  // 9
    };

    int commonPinStates[3][3] = {
      {1, 0, 0}, // habilitar el dígito 1
      {0, 1, 0}, // habilitar el dígito 2
      {0, 0, 1}  // habilitar el dígito 3
    };

    for (int n = 0; n < 3; n++) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(commonPins[i], commonPinStates[n][i]);
        (n == 1) ? digitalWrite(Dot, 1) : digitalWrite(Dot, 0);
      }

      for (int j = 0; j < 7; j++) {
        digitalWrite(segmentPins[j], number[placeValuesofTemp[n]][j]);
      }

      delay(sevSegRefresh);

      for (int i = 0; i < 7; i++) {
        digitalWrite(segmentPins[i], LOW); // Apagar todos los segmentos antes de cambiar de visualización
      }
    }

    // Control de color del LED RGB según la temperatura
    if (TempC_LM35 < 37.0) {
      ledcWrite(LEDC_CHANNEL_0, 0);     // Apagar LED rojo
      ledcWrite(LEDC_CHANNEL_1, 255);   // Encender LED verde
      ledcWrite(LEDC_CHANNEL_2, 0);     // Apagar LED azul
    } else if (TempC_LM35 >= 37.0 && TempC_LM35 <= 37.5) {
      ledcWrite(LEDC_CHANNEL_0, 255);   // Encender LED rojo
      ledcWrite(LEDC_CHANNEL_1, 255);   // Encender LED verde
      ledcWrite(LEDC_CHANNEL_2, 0);     // Apagar LED azul
    } else {
      ledcWrite(LEDC_CHANNEL_0, 255);   // Encender LED rojo
      ledcWrite(LEDC_CHANNEL_1, 0);     // Apagar LED verde
      ledcWrite(LEDC_CHANNEL_2, 0);     // Apagar LED azul
    }

    // Control del servo motor
    int servoPosition = map(TempC_LM35, 25, 40, 0, 180); // Ajuste para el rango de 25 a 40 grados
    ledcWrite(3, servoPosition);

  } else {
    // Apagar todos los segmentos si la temperatura no se ha tomado
    // o si los displays no están encendidos
    for (int i = 0; i < 7; i++) {
      digitalWrite(segmentPins[i], LOW);
    }
    digitalWrite(Dot, LOW);

    // Apagar el LED RGB si la temperatura no se ha tomado
    ledcWrite(LEDC_CHANNEL_0, 0);
    ledcWrite(LEDC_CHANNEL_1, 0);
    ledcWrite(LEDC_CHANNEL_2, 0);

    // Detener el servo si la temperatura no se ha tomado
    ledcWrite(3, 0);
  }

  // Leer y mostrar la temperatura si el botón se ha presionado
  if (displaysOn && !temperatureTaken && reading == LOW) {
    int SNLM35_Raw = analogRead(SNLM35);
    float Voltage = readADC_Cal(SNLM35_Raw);
    TempC_LM35 = Voltage / 10;

    temp = TempC_LM35 * 100;

    placeValuesofTemp[3] = ((temp) / 1) % 10;
    placeValuesofTemp[2] = ((temp) / 10) % 10;
    placeValuesofTemp[1] = ((temp) / 100) % 10;
    placeValuesofTemp[0] = ((temp) / 1000) % 10;

    Serial.print("Temperatura: ");
    Serial.println(TempC_LM35);
    temperatureTaken = true;
  }
}
