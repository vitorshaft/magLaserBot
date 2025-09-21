#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <QMC5883LCompass.h>
#include <math.h>

// ==================== CONFIGURAÇÕES DE HARDWARE ====================

// Display OLED
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1

// Driver L298N
#define AIN1 14
#define AIN2 27
#define PWMA 12
#define BIN1 33
#define BIN2 32
#define PWMB 13

// Encoders
#define ENC_ESQ_A 25
#define ENC_ESQ_B 26
#define ENC_DIR_A 35
#define ENC_DIR_B 34

// Constantes do robô
const float WHEEL_DIAMETER = 40.0;  // Diâmetro da roda em mm
const float WHEEL_BASE = 120.0;      // Distância entre as rodas em mm
const int PULSES_PER_REVOLUTION = 1000;  // Pulsos do encoder por volta completa

// Configurações PWM
const int PWM_FREQ = 5000;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int PWM_RESOLUTION = 8;

// ==================== VARIÁVEIS GLOBAIS ====================

// Sensores
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
QMC5883LCompass compass;

// Encoders
volatile long encoderEsqCount = 0;
volatile long encoderDirCount = 0;

// Navegação
float robotX = 0.0;      // Posição X atual (mm)
float robotY = 0.0;      // Posição Y atual (mm)
float robotTheta = 0.0;  // Orientação atual (radianos)
long lastLeftCount = 0;
long lastRightCount = 0;

// Mapeamento
const int MAP_SIZE = 50;  // Tamanho do mapa (grid 50x50)
int occupancyGrid[MAP_SIZE][MAP_SIZE] = {0};  // 0=livre, 1=ocupado, 2=desconhecido
float mapScale = 20.0;    // mm por célula do grid

// Constantes
const float DEG2RAD = 0.017453292519943295f;
const float RAD2DEG = 57.29577951308232f;
const float MAGNETIC_DECLINATION = -21.0f;

// ==================== INTERRUPÇÕES DOS ENCODERS ====================

void IRAM_ATTR ISR_EncoderEsq() {
  if (digitalRead(ENC_ESQ_B) == HIGH) {
    encoderEsqCount++;
  } else {
    encoderEsqCount--;
  }
}

void IRAM_ATTR ISR_EncoderDir() {
  if (digitalRead(ENC_DIR_B) == HIGH) {
    encoderDirCount--;
  } else {
    encoderDirCount++;
  }
}

// ==================== CONTROLE DOS MOTORES ====================

void setupMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);
}

void moveMotor(int motor, int direction, int speed) {
  bool in1 = LOW, in2 = LOW;
  
  if (direction == 1) {
    in1 = HIGH;
    in2 = LOW;
  } else if (direction == -1) {
    in1 = LOW;
    in2 = HIGH;
  }
  
  if (motor == 0) {
    digitalWrite(AIN1, in1);
    digitalWrite(AIN2, in2);
    ledcWrite(PWM_CHANNEL_A, speed);
  } else {
    digitalWrite(BIN1, in1);
    digitalWrite(BIN2, in2);
    ledcWrite(PWM_CHANNEL_B, speed);
  }
}

void stopMotors() {
  moveMotor(0, 0, 0);
  moveMotor(1, 0, 0);
}

// ==================== ODOMETRIA E LOCALIZAÇÃO ====================

void updateOdometry() {
  long leftDelta = encoderEsqCount - lastLeftCount;
  long rightDelta = encoderDirCount - lastRightCount;
  
  // Calcular distância percorrida por cada roda (mm)
  float distLeft = (leftDelta * PI * WHEEL_DIAMETER) / PULSES_PER_REVOLUTION;
  float distRight = (rightDelta * PI * WHEEL_DIAMETER) / PULSES_PER_REVOLUTION;
  
  // Calcular mudança na posição e orientação
  float deltaDist = (distLeft + distRight) / 2.0;
  float deltaTheta = (distRight - distLeft) / WHEEL_BASE;
  
  // Atualizar pose do robô
  robotTheta += deltaTheta;
  robotX += deltaDist * cos(robotTheta);
  robotY += deltaDist * sin(robotTheta);
  
  // Normalizar orientação
  if (robotTheta > 2 * PI) robotTheta -= 2 * PI;
  if (robotTheta < 0) robotTheta += 2 * PI;
  
  lastLeftCount = encoderEsqCount;
  lastRightCount = encoderDirCount;
}

// ==================== SENSORES E MAPEAMENTO ====================

bool readCompass(float &heading) {
  compass.read();
  int azimuth = compass.getAzimuth();
  
  heading = azimuth + MAGNETIC_DECLINATION;
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;
  
  heading *= DEG2RAD;
  return true;
}

bool readDistance(int &distance) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus == 4) {
    distance = -1;
    return false;
  }
  
  distance = measure.RangeMilliMeter;
  return (distance > 0 && distance < 2000);
}

void updateOccupancyGrid(float sensorAngle, int distance) {
  if (distance < 0 || distance > 1500) return;
  
  // Calcular posição do obstáculo no sistema de coordenadas do robô
  float obstacleX = distance * cos(sensorAngle);
  float obstacleY = distance * sin(sensorAngle);
  
  // Converter para coordenadas globais
  float globalX = robotX + obstacleX * cos(robotTheta) - obstacleY * sin(robotTheta);
  float globalY = robotY + obstacleX * sin(robotTheta) + obstacleY * cos(robotTheta);
  
  // Converter para coordenadas do grid
  int gridX = MAP_SIZE / 2 + (int)(globalX / mapScale);
  int gridY = MAP_SIZE / 2 + (int)(globalY / mapScale);
  
  // Atualizar grid (se dentro dos limites)
  if (gridX >= 0 && gridX < MAP_SIZE && gridY >= 0 && gridY < MAP_SIZE) {
    occupancyGrid[gridY][gridX] = 1;  // Marcado como ocupado
  }
}

// ==================== VISUALIZAÇÃO NO DISPLAY ====================

void drawMap() {
  display.clearDisplay();
  
  // Desenhar grid de ocupação
  int displayScale = 2;  // Escala para visualização no display
  for (int y = 0; y < MAP_SIZE; y++) {
    for (int x = 0; x < MAP_SIZE; x++) {
      if (occupancyGrid[y][x] == 1) {
        int displayX = x / displayScale;
        int displayY = y / displayScale;
        if (displayX < OLED_WIDTH && displayY < OLED_HEIGHT) {
          display.drawPixel(displayX, displayY, SSD1306_WHITE);
        }
      }
    }
  }
  
  // Desenhar posição do robô
  int robotDisplayX = ((MAP_SIZE / 2 + (int)(robotX / mapScale)) / displayScale)+70;
  int robotDisplayY = ((MAP_SIZE / 2 + (int)(robotY / mapScale)) / displayScale)+8;
  
  if (robotDisplayX >= 0 && robotDisplayX < OLED_WIDTH && 
      robotDisplayY >= 0 && robotDisplayY < OLED_HEIGHT) {
    display.fillCircle(robotDisplayX, robotDisplayY, 2, SSD1306_WHITE);
    
    // Desenhar orientação
    int dirX = robotDisplayX + 5 * cos(robotTheta);
    int dirY = robotDisplayY + 5 * sin(robotTheta);
    display.drawLine(robotDisplayX, robotDisplayY, dirX, dirY, SSD1306_WHITE);
  }
  
  // Informações de status
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("X:%.0f Y:%.0f", robotX, robotY);
  display.setCursor(0, 10);
  display.printf("θ:%.0f°", robotTheta * RAD2DEG);
  
  display.display();
}

// ==================== COMPORTAMENTO DO ROBÔ ====================

void avoidObstacles() {
  int distance;
  float heading;
  
  readCompass(heading);
  if (readDistance(distance) && distance < 200) {
    // Obstáculo detectado - girar aleatoriamente
    stopMotors();
    delay(200);
    
    // Girar para a direita ou esquerda
    if (random(2) == 0) {
      moveMotor(0, -1, 150);
      moveMotor(1, 1, 150);
    } else {
      moveMotor(0, 1, 150);
      moveMotor(1, -1, 150);
    }
    
    delay(500 + random(1000));
    stopMotors();
  } else {
    // Navegação normal - seguir em frente
    moveMotor(0, 1, 150);
    moveMotor(1, 1, 150);
  }
}

// ==================== SETUP E LOOP PRINCIPAL ====================

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  randomSeed(analogRead(0));
  
  // Inicializar display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Display initialization failed!");
  }
  
  // Inicializar sensor de distância
  if (!lox.begin()) {
    Serial.println("VL53L0X initialization failed!");
  }
  
  // Inicializar bússola
  compass.init();
  compass.setSmoothing(10, true);
  
  // Inicializar motores
  setupMotors();
  
  // Configurar encoders
  pinMode(ENC_ESQ_A, INPUT_PULLUP);
  pinMode(ENC_ESQ_B, INPUT_PULLUP);
  pinMode(ENC_DIR_A, INPUT_PULLUP);
  pinMode(ENC_DIR_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_ESQ_A), ISR_EncoderEsq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DIR_A), ISR_EncoderDir, RISING);
  
  // Mensagem inicial
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("SLAM Robot Ready");
  display.display();
  delay(1000);
}

void loop() {
  // Atualizar odometria
  updateOdometry();
  
  // Ler sensores
  float heading;
  int distance;
  
  readCompass(heading);
  if (readDistance(distance)) {
    // Atualizar mapa com a leitura do sensor
    updateOccupancyGrid(0, distance);  // Sensor está apontando para frente (0 rad)
  }
  
  // Tomar decisão de navegação
  avoidObstacles();
  
  // Atualizar display
  drawMap();
  
  // Log para serial
  Serial.printf("Pos: %.1f, %.1f, %.1f° | Dist: %d mm\n", 
                robotX, robotY, robotTheta * RAD2DEG, distance);
  
  delay(100);
}