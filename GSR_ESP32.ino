#include <WiFi.h>
#include <WebServer.h>  // Cambiamos a WebServer sincrónico
#include <LittleFS.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/sockets.h>

const char* ssid = "Luisifer_2.4Gnormal";
const char* password = "cradleoffilth21";

WebServer server(80);  // Cambio a WebServer
const int GSR = 32;

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
  Serial.println(gsr_average);
  return String(gsr_average);
}

void handleRoot() {
  if (LittleFS.exists("/index.html")) {
    File file = LittleFS.open("/index.html", "r");
    server.streamFile(file, "text/html");
    file.close();
  } else {
    server.send(200, "text/plain", "Archivo index.html no encontrado");
  }
}

void handleHumanResistance() {
  server.send(200, "text/plain", sensorReading());
}

void handleNotFound() {
  server.send(404, "text/plain", "Endpoint no encontrado");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nIniciando...");
  
  // Configuración crítica del stack de red
  esp_netif_init();
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setSleep(false);

  // Inicializa LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Error con LittleFS");
    while (1) delay(1000);
  }

  // Conexión WiFi con timeout
  WiFi.begin(ssid, password);
  Serial.print("Conectando WiFi");
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado! IP: " + WiFi.localIP().toString());
    delay(1000);  // Pequeña espera adicional

    // Configuración de endpoints
    server.on("/", handleRoot);
    server.on("/humanResistance", handleHumanResistance);
    server.onNotFound(handleNotFound);
    
    server.begin();
    Serial.println("Servidor HTTP iniciado");
  } else {
    Serial.println("\nFalló la conexión WiFi. Reiniciando...");
    delay(2000);
    ESP.restart();
  }
}

void loop() {
  server.handleClient();  // Maneja las peticiones HTTP
  
  // Verificación periódica de WiFi
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi desconectado. Reconectando...");
      WiFi.reconnect();
    }
  }
  delay(1);  // Pequeña pausa para liberar CPU
}