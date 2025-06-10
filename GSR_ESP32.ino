#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/sockets.h>
#include <LiquidCrystal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Configuración de red
const char* ssid = "Luisifer_2.4Gnormal";
const char* password = "cradleoffilth21";

// Configuración de hardware
const int GSR = 32;           // Sensor GSR (entrada de referencia)
const int encoder_pot = 35;    // Sensor de feedback (posición)
const int PWMPin = 33;        // Pin PWM para el motor
const int DirPin1 = 27;       // Dirección motor pin 1
const int DirPin2 = 14;       // Dirección motor pin 2
const int rs = 15, en = 2, d4 = 4, d5 = 16, d6 = 17, d7 = 5; // Pines LCD

// Objetos globales
WebServer server(80);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Variables compartidas (protegidas por semáforos)
SemaphoreHandle_t xMutex;
volatile int gsrValue = 0;       // Valor GSR (0-1023)
volatile int encoderValue = 0;    // Valor encoder (0-1023)
volatile float motorVoltage = 0;  // Voltaje de salida PID
volatile bool wifiConnected = false;

// Parámetros PID, estos parametros ajustan la respuesta del sistema
const float kp = 0.2;// mientras mas alto sea este valor, oscilara mas tu respuesta
const float ki = 0.00000;//mientras mas alto sea este valor se reduce el error en estado estacionario
const float kd = 2.00;//mientras mas alto sea este valor, se reduce la banda muerta entre el SP y mi variable
const float Vmax = 5;
const float Vmin = -5;

// Variables de estado PID
volatile unsigned long t_prev = 0;
volatile float e_prev = 0;
volatile float inte_prev = 0;

// Declaraciones de funciones
void wifiTask(void *pvParameters);
void sensorTask(void *pvParameters);
void pidTask(void *pvParameters);
void motorTask(void *pvParameters);
void lcdTask(void *pvParameters);
void serverTask(void *pvParameters);
String sensorReading();
void initializeLittleFS();
void initializeLCD();
void connectToWiFi();
void initializeWebServer();
void WriteDriverVoltage(float V, float Vmax);
int scaleTo1023(int value);

// Función para escalar ADC de 0-4095 a 0-1023
int scaleTo1023(int value) {
  return map(value, 0, 4095, 0, 1023);
}

// Función para controlar el motor
void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) PWMval = 255;
  
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  }
  else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  }
  else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin, PWMval);
}

String sensorReading() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    int sensorValue = analogRead(GSR);
    if (sensorValue == 0 || sensorValue == 4095) {
      Serial.println("Error lectura sensor");
      return "ERROR";
    }
    sum += sensorValue;
    delay(5);
  }
  int gsr_average = sum / 10;
  return String(scaleTo1023(gsr_average)); // Convertir a 0-1023
}

void initializeLCD() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Iniciando sistema");
  delay(1000);
}

void initializeLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("Error con LittleFS");
    lcd.clear();
    lcd.print("Error LittleFS");
    while (1) delay(1000);
  }
}

void connectToWiFi() {
  // Configuración de red
  esp_netif_init();
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setSleep(false);

  // Conexión WiFi
  WiFi.begin(ssid, password);
  lcd.clear();
  lcd.print("Conectando WiFi...");

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) {
    delay(500);
    Serial.print(".");
    static int dots = 0;
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(dots % 6, 1);
    lcd.print(".");
    dots++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    lcd.clear();
    lcd.print("WiFi Conectado");
    lcd.setCursor(0, 1);
    lcd.print("IP: ");
    lcd.print(WiFi.localIP());
    delay(2000);
  } else {
    lcd.clear();
    lcd.print("Error WiFi");
    lcd.setCursor(0, 1);
    lcd.print("Reiniciando...");
    delay(2000);
    ESP.restart();
  }
}

void initializeWebServer() {
  server.on("/", []() {
    if (LittleFS.exists("/index.html")) {
      File file = LittleFS.open("/index.html", "r");
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(200, "text/plain", "Archivo no encontrado");
    }
  });
  
  server.on("/humanResistance", []() {
    String reading;
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      reading = String(gsrValue);
      xSemaphoreGive(xMutex);
    }
    server.send(200, "text/plain", reading);
  });
  server.begin();
}

// Tarea WiFi
void wifiTask(void *pvParameters) {
  connectToWiFi();
  initializeWebServer();
  
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      connectToWiFi();
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

// Tarea Sensor
void sensorTask(void *pvParameters) {
  for (;;) {
    int gsr = sensorReading().toInt();
    int encoder = scaleTo1023(analogRead(encoder_pot));
    
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      gsrValue = gsr;
      encoderValue = 1023 - encoder; // Relación inversa
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Tarea PID
void pidTask(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      unsigned long t = millis();
      int dt = t - t_prev;
      
      if (dt > 0) {
        float Theta = encoderValue;
        float Theta_d = gsrValue;
        float e = Theta_d - Theta;
        float inte = inte_prev + (dt * (e + e_prev) / 2);
        
        // Cálculo PID
        float V = kp * e + ki * inte + (kd * (e - e_prev) / dt);
        
        // Anti-windup
        if (V > Vmax) {
          V = Vmax;
          inte = inte_prev;
        }
        else if (V < Vmin) {
          V = Vmin;
          inte = inte_prev;
        }
        
        motorVoltage = V;
        t_prev = t;
        e_prev = e;
        inte_prev = inte;
      }
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Tarea Motor
void motorTask(void *pvParameters) {
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  pinMode(PWMPin, OUTPUT);
  
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      WriteDriverVoltage(motorVoltage, Vmax);
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// Tarea LCD
void lcdTask(void *pvParameters) {
  for (;;) {
    int valor, encoder;
    float voltage;
    
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      valor = gsrValue;
      encoder = encoderValue;
      voltage = motorVoltage;
      xSemaphoreGive(xMutex);
    }
    
    String estado;
    if (valor == 0) estado = "Error Sensor";
    else if (valor < 60) estado = "Fuera Rango";
    else if (valor < 100) estado = "Relajacion";
    else if (valor < 300) estado = "Neutro";
    else if (valor < 500) estado = "Estres Leve";
    else if (valor < 700) estado = "Estres Alto";
    else estado = "Dolor";

    lcd.clear();
    lcd.print("GSR:");
    lcd.print(valor);
    lcd.print(" E:");
    lcd.print(encoder);
    
    lcd.setCursor(0, 1);
    lcd.print(estado);
    lcd.print(" ");
    lcd.print(voltage, 1);
    lcd.print("V");
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Tarea Servidor
void serverTask(void *pvParameters) {
  for (;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Inicializar LCD
  initializeLCD();
  Serial.println("\nIniciando...");

  // Inicializar LittleFS
  initializeLittleFS();

  // Crear mutex
  xMutex = xSemaphoreCreateMutex();

  // Crear tareas
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(pidTask, "PIDTask", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(lcdTask, "LCDTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(serverTask, "ServerTask", 4096, NULL, 1, NULL, 0);

  vTaskDelete(NULL);
}

void loop() {}