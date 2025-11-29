// ===============================
//   CONTROL PID
//
//   Prueba con un sensor ultrasonico
// ===============================

// ---- Pines ----
const int trigPin = A0; //Pin TRIG del sensor
const int echoPin = A1; // Pin Echo del sensor
const int motorPin = 9; // Pin digital de señal para el motor

// ---- Parámetros PID ----
float Kp = 1.2;
float Ki = 0.4;
float Kd = 0.2;

// ---- Variables PID ----
float error = 0;
float prevError = 0;
float integral = 0;
float derivada = 0;

float output = 0;

// ---- Tiempo de muestreo ----
unsigned long lastTime = 0;
const float Ts = 0.05; // 50 ms

// ---- Setpoint ----
float distancia_objetivo = 20.0; // cm deseados

// ---- Tolerancia ----
float tolerancia = 1.5;

// ===============================
// Lecturas del sensor ultrasonico
// ===============================
float medirDistancia()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float cm = duration * 0.0343 / 2;
  return cm;
}

// ===============================
// Funcion del controlador PID
// ===============================
float PID(float setpoint, float medida)
{
  error = setpoint - medida;

  integral += error * Ts;
  derivada = (error - prevError) / Ts;

  float u = Kp * error + Ki * integral + Kd * derivada;

  prevError = error;

  // saturación
  u = constrain(u, 0, 255);
  return u;
}


void setup()
{
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin, OUTPUT);
}

void loop()
{
  unsigned long now = millis();

  // Ejecutar PID cada Ts
  if (now - lastTime >= Ts * 1000)
  {
    lastTime = now;

    float distancia = medirDistancia();

    float control = PID(distancia_objetivo, distancia);

    analogWrite(motorPin, control);

    Serial.print("Distancia: ");
    Serial.print(distancia);
    Serial.print("   |   Control: ");
    Serial.println(control);

    // Si ya llegó se detiene
    if (abs(distancia - distancia_objetivo) <= tolerancia)
    {
      analogWrite(motorPin, 0);
      Serial.println(" La distancia fue alcanzada.");
    }
  }
}