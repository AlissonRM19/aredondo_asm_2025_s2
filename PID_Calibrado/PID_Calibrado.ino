// ===============================
//   PID con Sensor Ultrasónico HC-SR04
//   Motor DC con Puente H L9110S
// ===============================

// ---- Pines Sensor Ultrasónico ----
const int trigPin = 7;
const int echoPin = 8;

// ---- Pines Puente H L9110S ----
// El L9110S tiene 2 pines por motor (A-IA, A-IB)
const int motorA_IA = 9;   // PWM - Dirección adelante
const int motorA_IB = 10;  // PWM - Dirección atrás

// ---- PID ----
float Kp = 15;   // Aumentado para respuesta más rápida 5 base
float Ki = 10;
float Kd = 5;   // Aumentado para reducir oscilaciones 0.8 base

float error = 0, prevError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// ---- Tiempo ----
unsigned long lastTime = 0;
const float Ts = 0.05;  // 50ms (20Hz)

// ---- Control ----
float distancia_objetivo = 7;  // cm desde el sensor
float distancia_actual = 0.0;
float tolerancia = 1;  // cm

bool objetivoAlcanzado = false;

// ===============================
// Leer Sensor Ultrasónico
// ===============================

float leerDistancia()
{
  // Enviar pulso de 10us
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Leer tiempo de retorno
  long duracion = pulseIn(echoPin, HIGH, 30000);  // timeout 30ms
  
  // Calcular distancia en cm
  // velocidad sonido = 343 m/s = 0.0343 cm/us
  // distancia = (tiempo * velocidad) / 2
  float distancia = duracion * 0.0343 / 2.0;
  
  // Filtrar lecturas inválidas
  if (distancia <= 0 || distancia > 400) {
    return distancia_actual;  // mantener lectura anterior
  }
  
  return distancia;
}

// ===============================
// Control Motor con L9110S
// ===============================

void moverMotor(float pwm)
{
  // El L9110S controla dirección con dos señales PWM:
  // - Adelante: A-IA=PWM, A-IB=0
  // - Atrás: A-IA=0, A-IB=PWM
  // - Stop: ambos a 0
  
  int velocidad = abs(pwm);
  velocidad = constrain(velocidad, 180, 255);
  
  if (pwm > 0) {
    // Mover adelante (acercarse al objetivo)
    analogWrite(motorA_IA, velocidad);
    analogWrite(motorA_IB, 0);
  }
  else if (pwm < 0) {
    // Mover atrás (alejarse del objetivo)
    analogWrite(motorA_IA, 0);
    analogWrite(motorA_IB, velocidad);
  }
  else {
    // Detener
    analogWrite(motorA_IA, 0);
    analogWrite(motorA_IB, 0);
  }
}

// ===============================
// PID
// ===============================

float PID(float setpoint, float medida)
{
  error = setpoint - medida;
  
  // Solo integrar si no estamos en el objetivo
  if (abs(error) > tolerancia) {
    integral += error * Ts;
    integral = constrain(integral, -100, 100);  // anti-windup
  } else {
    integral = 0;  // resetear cuando alcanzamos objetivo
  }
  
  derivative = (error - prevError) / Ts;
  
  float u = Kp * error + Ki * integral + Kd * derivative;
  
  prevError = error;
  
  // Limitar salida PWM
  return constrain(u, -255, 255);
}

// ===============================
// Setup
// ===============================

void setup()
{
  Serial.begin(9600);
  
  // Configurar pines del sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Configurar pines del motor
  pinMode(motorA_IA, OUTPUT);
  pinMode(motorA_IB, OUTPUT);
  
  // Detener motor al inicio
  moverMotor(0);
  
  Serial.println("=== Sistema PID con Ultrasonico ===");
  Serial.print("Objetivo: ");
  Serial.print(distancia_objetivo);
  Serial.println(" cm");
  Serial.println("===================================");
  
  delay(1000);  // Espera inicial

  
}

// ===============================
// Loop
// ===============================

void loop()
{
  unsigned long now = millis();
  
  if (now - lastTime >= Ts * 1000)
  {
    lastTime = now;
    
    // Leer distancia real del sensor
    distancia_actual = leerDistancia();
    
    // Si ya alcanzó el objetivo, mantener detenido
    if (abs(distancia_actual - distancia_objetivo) <= tolerancia)
    {
      if (!objetivoAlcanzado) {
        objetivoAlcanzado = true;
        moverMotor(0);
        integral = 0;  // resetear integral
        Serial.println(">>> OBJETIVO ALCANZADO <<<");
        delay(5000);
        integral = 0;
        distancia_objetivo = 23;

        
      }
      return;  // No hacer más cálculos
    }
    else
    {
      objetivoAlcanzado = false;
    }
    
    // Calcular control PID
    float control = PID(distancia_objetivo, distancia_actual);
    
    // Aplicar control al motor
    moverMotor(control);
    
    // Mostrar información
    Serial.print("Dist: ");
    Serial.print(distancia_actual, 1);
    Serial.print(" cm | Error: ");
    Serial.print(error, 1);
    Serial.print(" | PWM: ");
    Serial.print(control, 0);
    Serial.print(" | I: ");
    Serial.println(integral, 1);
  }
}