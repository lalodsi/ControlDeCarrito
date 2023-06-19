
// Entradas digitales
#define PIN_ENCODER_I 2
#define PIN_ENCODER_D 3
#define PIN_MOTOR_A_1 9
#define PIN_MOTOR_A_2 8
#define PIN_MOTOR_B_1 7
#define PIN_MOTOR_B_2 6

// Matematicas y medidas
#define PI 3.1416
#define WHEEL_DIAMETER 71.4

volatile byte pulsesI = 0;       // Número de pulsos leidos por el Arduino en un segundo
volatile byte pulsesD = 0;       // Número de pulsos leidos por el Arduino en un segundo
unsigned int rpmI = 0;           // Revoluciones por minuto calculadas.
unsigned int rpmD = 0;           // Revoluciones por minuto calculadas.
float velocityI = 0;                 //Velocidad en [Km/h]
float velocityD = 0;                 //Velocidad en [Km/h]
// unsigned long timeold = 0;  // Tiempo transcurrido
double timeold = 0;  // Tiempo transcurrido
const int wheel_diameter = 64;   // Diámetro de la rueda pequeña[mm]
static volatile unsigned long debounceI = 0; // Tiempo del rebote.
static volatile unsigned long debounceD = 0; // Tiempo del rebote.
static volatile unsigned long deltaX = 0; // Tiempo de transcurrido entre pulsos.

// Constantes Trayectoria
double T = 10;
int r = 1; // Radio del circulo
double f = 1/T;

// Constantes Mapeo
double L = 1.5;

// Variables de control
double wd_d = 0;
double wi_d = 0;
double wd = 0;
double wi = 0;
double Xp_1;
double Yp_1;
double thetap_1;
double Xp;
double Yp;
double thetap;
double X;
double Y;
double theta;

// Variables trayectoria
double thetad = 0;
double thetadp = 0;
double Xd = 0;
double Yd = 0;
double Xdp = 0;
double Ydp = 0;

// Variables control y planta
double V = 0;
double W = 0;


// Variables Externas
#define SLOTS_ENCODER 20

void setup()
{
    pinMode(PIN_ENCODER_I, INPUT);
    pinMode(PIN_ENCODER_D, INPUT);
    pinMode(PIN_MOTOR_A_1, OUTPUT);
    pinMode(PIN_MOTOR_A_2, OUTPUT);
    pinMode(PIN_MOTOR_B_1, OUTPUT);
    pinMode(PIN_MOTOR_B_2, OUTPUT);

    Serial.begin(9600);

    Xp = 0;
    Yp = 0;
    thetap = 0;
    X = 0;
    Y = 0;
    theta = 0;

    pulsesI = 0;
    pulsesD = 0;
    rpmI = 0;
    rpmD = 0;
    timeold = 0;

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_I), counterI, RISING); // Configuración de la interrupción 0, donde esta conectado. 
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_D), counterD, RISING); // Configuración de la interrupción 0, donde esta conectado. 
}

void loop()
{
    if (millis() - timeold >= 1000){  // Se actualiza cada segundo
      noInterrupts(); //Don't process interrupts during calculations // Desconectamos la interrupción para que no actué en esta parte del programa.
        // rpmI = ((float)pulsesI / (float)SLOTS_ENCODER ) * 60 * 4 ; // Calculamos las revoluciones por minuto
        // rpmD = ((float)pulsesD / (float)SLOTS_ENCODER ) * 60 * 4 ; // Calculamos las revoluciones por minuto
        wd = pulsesD * 2.0 * PI;
        wi = pulsesI * 2.0 * PI;
        // velocityI = rpmI * PI * WHEEL_DIAMETER * 60 / 1000000; // Cálculo de la velocidad en [Km/h] 
        // velocityD = rpmI * PI * WHEEL_DIAMETER * 60 / 1000000; // Cálculo de la velocidad en [Km/h] 
        timeold = millis() / 100000 ; // Almacenamos el tiempo actual.

        // Bloques
        trayectoria(timeold);
        controlYPlanta();
        mapeo();
        // planta();
        //Serial.print("Tiempo transcurrido: ");
        //Serial.print(millis()/1000); 
        // Serial.print("\nVelocidad en RPMI: ");
        // Serial.print(rpmI,DEC);
        // Serial.print("\nVelocidad en RPMD: ");
        // Serial.print(rpmD,DEC);
        // Serial.print("\nPulsos Realizados Izquierda: ");
        // Serial.print(pulsesI,DEC);
        // Serial.print("\nPulsos Realizados Derecha: ");
        // Serial.print(pulsesD,DEC);
        // Serial.print("\nVelocidad Izquierda: ");
        // Serial.print(velocityI, 2);
        // Serial.print("\nVelocidad Derecha: ");
        // Serial.print(velocityD, 2);
        // Serial.print("\nTiempo Izquierda: ");
        // Serial.print(debounceI, 2);
        // Serial.print("\nTiempo Derecha: ");
        // Serial.println(debounceD, 2);
        // Serial.print("\n\n\n\n\n\n\n\n\n");
        Serial.print(wd_d);
        Serial.print(",");
        Serial.print(wd);
        Serial.print(",");
        Serial.print(wi_d);
        Serial.print(",");
        Serial.print(wi);

        // Serial.print(", ");
        Serial.println(",");
        //Serial.println(velocityI);
        pulsesI = 0;  // Inicializamos los pulsos.
        pulsesD = 0;  // Inicializamos los pulsos.
      interrupts(); // Restart the interrupt processing // Reiniciamos la interrupción
   }
}

void counterI(){
    // Serial.println(debounceI);
    if (digitalRead(PIN_ENCODER_I))
    {
        delay(50);
        if(digitalRead(PIN_ENCODER_I))
        {
            // Vuelve a comprobar que el encoder envia una señal buena y luego
            // comprueba que el tiempo es superior a 50 microsegundos y vuelve
            // a comprobar que la señal es correcta.
            moverMotorI(wi_d, wi);
            deltaX = micros() - debounceI;
            pulsesI++;
        }
    }
    integral(deltaX);
    debounceI = micros();
}

void counterD(){
    if (digitalRead(PIN_ENCODER_D))
    {
        delay(50);
        if(digitalRead(PIN_ENCODER_D))
        {
            // Vuelve a comprobar que el encoder envia una señal buena y luego
            // comprueba que el tiempo es superior a 50 microsegundos y vuelve
            // a comprobar que la señal es correcta.
            moverMotorD(wd_d, wd);
            deltaX = micros() - debounceD;
            pulsesD++;
        }
    }
    integral(deltaX);
    debounceD = micros();
}

/**
 * Genera la trayectoria deseada
*/
void trayectoria(double time){
    thetad = 2*PI*f*time;
    thetadp = 2*PI*f;

    Xd = r*sin(thetad);
    Yd = -r*cos(thetad);

    Xdp = r*thetadp*cos(thetad);
    Ydp = r*thetadp*sin(thetad);

}

void controlYPlanta(){
    double k1 = 20;
    double k2 = 20;
    double d = 0.01f;

    double V1aux = Xdp + k1*(Xd - X);
    double V2aux = Ydp + k2*(Yd - Y);

    V = V1aux*cos(thetad) + V2aux*sin(thetad);
    W = -V1aux*sin(thetad)/d + V2aux*cos(thetad)/d;
}

void mapeo(){
    // double T[2][2] = {{r/2, r/2}, {r/(2*L), r/(2*L)}} //r/2*[1 1; 1/L -1/L];
    double Tinv[2][2] = {{1/r, 1/(r*L)}, {1/r, -1/(r*L)}};

    wd_d = Tinv[0][0]*V + Tinv[0][1]*W;
    wi_d = Tinv[1][0]*V + Tinv[1][1]*W;
}

void planta(){
    // Valor anterior
    Xp_1 = Xp;
    Yp_1 = Yp;
    thetap_1 = thetap;
    // Nuevo valor
    Xp = (r/2)*(wd_d+wi_d)*cos(theta);
    Yp = (r/2)*(wd_d+wi_d)*sin(theta);
    thetap = (r/2*L)*(wd_d-wi_d);
}

void integral(double delta){
    X = (Xp - Xp_1)*delta;
    X = (Yp - Xp_1)*delta;
    theta = (thetap - thetap_1)*delta;
}

void moverMotorI(double velocidad_d, double velocidad_r){
    if (velocidad_d > velocidad_r) {
        digitalWrite(PIN_MOTOR_A_1, HIGH);
        digitalWrite(PIN_MOTOR_A_2, LOW);
    }
    else {
        digitalWrite(PIN_MOTOR_A_1, LOW);
        digitalWrite(PIN_MOTOR_A_2, LOW);
    }
}

void moverMotorD(double velocidad_d, double velocidad_r){
    if (velocidad_d > velocidad_r) {
        digitalWrite(PIN_MOTOR_B_1, HIGH);
        digitalWrite(PIN_MOTOR_B_2, LOW);
    }
    else {
        digitalWrite(PIN_MOTOR_B_1, LOW);
        digitalWrite(PIN_MOTOR_B_2, LOW);
    }
}
