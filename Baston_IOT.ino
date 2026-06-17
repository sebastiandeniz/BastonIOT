#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <VL53L1X.h>

// ==========================================
// 2. DEFINICIÓN DE PINES 
// ==========================================
// Comunicaciones
#define GPS_RX_PIN 27 
#define GPS_TX_PIN 33 
#define SIM_RX_PIN 16 
#define SIM_TX_PIN 17 

// Actuadores y Sensores
#define BTN_SOS 32
#define MOTOR_1 26
#define MOTOR_2 25
#define XSHUT_1 18
#define XSHUT_2 19
#define PIN_BATERIA 34

// ==========================================
// 3. CONFIGURACIÓN LÓGICA Y RED
// ==========================================
const int DISTANCIA_MAX = 1000; 
const int DISTANCIA_MIN = 300;  
const float FACTOR_CALIBRACION = 1.099;

const char apn[]      = "internet"; 
const char gprsUser[] = "";
const char gprsPass[] = "";
const char broker[]   = "test.mosquitto.org"; 
const char topicSOS[] = "baston/sos";

// ==========================================
// 4. OBJETOS GLOBALES
// ==========================================
HardwareSerial gpsSerial(1);
HardwareSerial simSerial(2);

TinyGPSPlus gps;
TinyGsm modem(simSerial);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

VL53L1X tof1;
VL53L1X tof2;

bool lastBtnState = HIGH;
unsigned long lastReconnectAttempt = 0;

// ==========================================
// 5. MÁQUINA DE ESTADOS: CONTROL DE MOTORES
// ==========================================
struct ControladorMotor {
  int pin;
  unsigned long tiempoAnterior;
  bool encendido;
  int tiempoPulso = 50; 

  void actualizar(int distancia) {
    unsigned long tiempoActual = millis();

    if (distancia == 0 || distancia > DISTANCIA_MAX) {
      digitalWrite(pin, LOW);
      encendido = false;
    }
    else if (distancia <= DISTANCIA_MIN) {
      digitalWrite(pin, HIGH);
      encendido = true;
    }
    else {
      int pausa = map(distancia, DISTANCIA_MIN, DISTANCIA_MAX, 50, 800);

      if (encendido) {
        if (tiempoActual - tiempoAnterior >= tiempoPulso) {
          encendido = false;
          tiempoAnterior = tiempoActual;
          digitalWrite(pin, LOW);
        }
      } else {
        if (tiempoActual - tiempoAnterior >= pausa) {
          encendido = true;
          tiempoAnterior = tiempoActual;
          digitalWrite(pin, HIGH);
        }
      }
    }
  }
};

ControladorMotor canal1 = {MOTOR_1, 0, false};
ControladorMotor canal2 = {MOTOR_2, 0, false};

int obtenerPorcentajeBateria() {
  long sumaADC = 0;
  for(int i = 0; i < 20; i++) {
    sumaADC += analogRead(PIN_BATERIA);
    delay(2); 
  }
  
  float lecturaPromedio = sumaADC / 20.0;
  float voltajePin = (lecturaPromedio / 4095.0) * 3.3;
  float voltajeBateria = (voltajePin * 2.0) * FACTOR_CALIBRACION;
  
  int porcentaje = map(voltajeBateria * 100, 320, 420, 0, 100);
  
  if (porcentaje > 100) porcentaje = 100;
  if (porcentaje < 0) porcentaje = 0;
  
  return porcentaje;
}

// ==========================================
// CLASE: FILTRO DE MEDIANA
// ==========================================
class FiltroMediana {
  private:
    static const int TAMANO = 10; 
    int lecturas[TAMANO];
    int indice = 0;

  public:
    FiltroMediana() {
      for (int i = 0; i < TAMANO; i++) {
        lecturas[i] = 1500; 
      }
    }

    int filtrar(int nuevaLectura) {
      lecturas[indice] = nuevaLectura;
      indice = (indice + 1) % TAMANO;

      int copia[TAMANO];
      for (int i = 0; i < TAMANO; i++) {
        copia[i] = lecturas[i];
      }

      for (int i = 0; i < TAMANO - 1; i++) {
        for (int j = i + 1; j < TAMANO; j++) {
          if (copia[i] > copia[j]) {
            int temp = copia[i];
            copia[i] = copia[j];
            copia[j] = temp;
          }
        }
      }
      return copia[TAMANO / 2];
    }
};

FiltroMediana filtroToF1;
FiltroMediana filtroToF2;

volatile int dist1_global = 1500;
volatile int dist2_global = 1500;

const int MAX_ERRORES_TOLERADOS = 5; 
int erroresContador1 = 0;
int erroresContador2 = 0;

bool revivirSensor(VL53L1X &tof, int pinXshut, byte direccionI2C) {
  Serial.printf("[ALERTA HARDWARE] Reiniciando en caliente sensor en pin XSHUT %d...\n", pinXshut);
  
  digitalWrite(pinXshut, LOW);
  delay(20); 
  
  digitalWrite(pinXshut, HIGH);
  delay(50); 
  
  tof.setTimeout(200);
  if (!tof.init()) {
    return false; 
  }
  
  if (direccionI2C != 0x29) {
    tof.setAddress(direccionI2C);
  }
  tof.setDistanceMode(VL53L1X::Medium);
  tof.startContinuous(100);
  
  Serial.printf("[SISTEMA ROBUSTO] Sensor restaurado con éxito en dirección 0x%X\n", direccionI2C);
  return true;
}

// ==========================================
// TAREA MULTIHILO DEL CORE 0
// ==========================================
void TareaLecturaSensores(void * pvParameters) {
  for(;;) {
    
    // --- EVALUAR SENSOR 1 ---
    if (tof1.dataReady()) {
      int lectura = tof1.read(false);
      if (tof1.timeoutOccurred() || lectura == 0) { 
        erroresContador1++;
      } else {
        erroresContador1 = 0; 
        dist1_global = filtroToF1.filtrar(lectura);
      }
    } else if (millis() % 500 == 0) { 
      erroresContador1++;
    }

    if (erroresContador1 >= MAX_ERRORES_TOLERADOS) {
      if (revivirSensor(tof1, XSHUT_1, 0x30)) {
        erroresContador1 = 0;
      } else {
        erroresContador1 = MAX_ERRORES_TOLERADOS - 2; 
      }
    }

    // --- EVALUAR SENSOR 2 ---
    if (tof2.dataReady()) {
      int lectura = tof2.read(false);
      if (tof2.timeoutOccurred() || lectura == 0) {
        erroresContador2++;
      } else {
        erroresContador2 = 0;
        dist2_global = filtroToF2.filtrar(lectura);
      }
    } else if (millis() % 500 == 0) {
      erroresContador2++;
    }

    if (erroresContador2 >= MAX_ERRORES_TOLERADOS) {
      if (revivirSensor(tof2, XSHUT_2, 0x31)) {
        erroresContador2 = 0;
      } else {
        erroresContador2 = MAX_ERRORES_TOLERADOS - 2;
      }
    }

    vTaskDelay(15 / portTICK_PERIOD_MS); 
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(100000); 
  
  delay(1000);
  Serial.println("\n=== INICIANDO BASTÓN INTELIGENTE ===");

  pinMode(BTN_SOS, INPUT_PULLUP);
  pinMode(MOTOR_1, OUTPUT); digitalWrite(MOTOR_1, LOW);
  pinMode(MOTOR_2, OUTPUT); digitalWrite(MOTOR_2, LOW);

  pinMode(XSHUT_1, OUTPUT); digitalWrite(XSHUT_1, LOW);
  pinMode(XSHUT_2, OUTPUT); digitalWrite(XSHUT_2, LOW);
  delay(10); 

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  simSerial.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

  // ----------------------------------------------------
  // FASE A: Inicializar Sensores ToF (I2C)
  // ----------------------------------------------------
  Serial.println("Inicializando Sensores ToF...");

  // --- ToF 1 ---
  digitalWrite(XSHUT_1, HIGH); 
  delay(150); 
  tof1.setTimeout(500);
  if (!tof1.init()) {
    Serial.println("[ERROR CRÍTICO] ToF 1 no responde.");
  } else {
    tof1.setAddress(0x30);
    tof1.setDistanceMode(VL53L1X::Medium);
    tof1.startContinuous(100); 
    Serial.println("ToF 1 [OK]");
  }

  // --- ToF 2 ---
  digitalWrite(XSHUT_2, HIGH); 
  delay(150);
  tof2.setTimeout(500);
  if (!tof2.init()) {
    Serial.println("[ERROR CRÍTICO] ToF 2 no responde.");
  } else {
    tof2.setAddress(0x31);
    tof2.setDistanceMode(VL53L1X::Medium);
    tof2.startContinuous(100);
    Serial.println("ToF 2 [OK]");
  }

  // ----------------------------------------------------
  // FASE B: Inicializar GSM y Red Móvil
  // ----------------------------------------------------
  Serial.println("Esperando a que el hardware GSM arranque...");
  delay(3000); 

  Serial.println("Sincronizando baudios GSM...");
  simSerial.println("AT"); delay(100);
  simSerial.println("AT"); delay(500);
  while(simSerial.available()) { simSerial.read(); } 

  Serial.println("Inicializando módem SIM800L...");
  if (!modem.restart()) {
    Serial.println("[ERROR] No se pudo reiniciar el módem");
  } else {
    Serial.print("Conectando a la red móvil...");
    if (!modem.waitForNetwork(60000L)) {
      Serial.println(" [FALLO]");
    } else {
      Serial.println(" [OK]");
      Serial.print("Conectando al APN (GPRS)...");
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println(" [FALLO]");
      } else {
        Serial.println(" [OK]");
      }
    }
  }

  // ----------------------------------------------------
  // FASE C: Configurar MQTT
  // ----------------------------------------------------
  mqtt.setServer(broker, 1883);
  Serial.println("=== SISTEMA LISTO Y EN BUCLE ===");
  Serial.println("Lanzando hilo de sensores al Core 0...");
  xTaskCreatePinnedToCore(
    TareaLecturaSensores, 
    "HiloToF",            
    4096,                 
    NULL,                 
    1,                    
    NULL,                 
    0                     
  );

  Serial.println("=== SISTEMA LISTO Y EN BUCLE ===");
}

// ==========================================
// LOOP PRINCIPAL (NO BLOQUEANTE)
// ==========================================
void loop() {
  // 1. GESTIÓN MQTT
  if (!mqtt.connected() && modem.isGprsConnected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000L) {
      lastReconnectAttempt = now;
      Serial.print("Conectando a MQTT...");
      String clientId = "BastonESP32-" + String(random(0xffff), HEX);
      
      if (mqtt.connect(clientId.c_str())) {
        Serial.println(" [OK]");
      } else {
        Serial.print(" [FALLO] Estado: "); Serial.println(mqtt.state());
      }
    }
  } else if (mqtt.connected()) {
    mqtt.loop(); 
  }

  // 2. PARSEO GPS 
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 3. LECTURA DE SENSORES Y CONTROL HÁPTICO
  canal1.actualizar(dist1_global);
  canal2.actualizar(dist2_global);

  // 4. BOTÓN SOS
  bool btnState = digitalRead(BTN_SOS);
  if (btnState == LOW && lastBtnState == HIGH) {
    Serial.println("\n[!] === ALERTA SOS DETECTADA ===");
    
    int nivelBateria = obtenerPorcentajeBateria();
    Serial.print("Nivel de batería actual: "); 
    Serial.print(nivelBateria); 
    Serial.println("%");
    
    digitalWrite(MOTOR_1, HIGH);
    digitalWrite(MOTOR_2, HIGH);
    
    String payload = "{\"alerta\":\"SOS\"";
    payload += ", \"bateria\":";
    payload += String(nivelBateria);
    
    if (gps.location.isValid() && gps.location.isUpdated()) {
      payload += ", \"lat\":"; payload += String(gps.location.lat(), 6);
      payload += ", \"lng\":"; payload += String(gps.location.lng(), 6);
    } else {
      payload += ", \"ubicacion\":\"Buscando satelites o sin fijar...\"";
    }
    payload += "}";

    Serial.print("Publicando en "); Serial.print(topicSOS);
    Serial.print(" -> "); Serial.println(payload);

    if (mqtt.connected()) {
      if(mqtt.publish(topicSOS, payload.c_str())) {
         Serial.println("[OK] Mensaje enviado correctamente.");
      } else {
         Serial.println("[ERROR] Fallo al publicar el mensaje.");
      }
    } else {
      Serial.println("[ERROR] Sin conexión al broker.");
    }
    
    delay(500); 
    
    digitalWrite(MOTOR_1, LOW);
    digitalWrite(MOTOR_2, LOW);
  }
  lastBtnState = btnState;
  
  // ==========================================
  // 5. MONITORIZACIÓN POR PUERTO SERIE
  // ==========================================
  static unsigned long ultimoPrint = 0; 
  unsigned long ahora = millis();

  if (ahora - ultimoPrint >= 500) {
    ultimoPrint = ahora;

    Serial.printf("ToF 1 (Pin 18): %-4d mm   |   ToF 2 (Pin 19): %-4d mm\n", 
                  dist1_global, 
                  dist2_global);
  }
}