
// Entradas digitales
#define PIN_ENCODER_I 2
#define PIN_ENCODER_D 3
#define PIN_MOTOR_A_1 6
#define PIN_MOTOR_A_2 7
#define PIN_MOTOR_B_1 8
#define PIN_MOTOR_B_2 9

// Matematicas y medidas
#define PI 3.1416
#define WHEEL_DIAMETER 71.4

volatile byte pulsesI = 0;       // Número de pulsos leidos por el Arduino en un segundo
volatile byte pulsesD = 0;       // Número de pulsos leidos por el Arduino en un segundo
unsigned int rpmI = 0;           // Revoluciones por minuto calculadas.
unsigned int rpmD = 0;           // Revoluciones por minuto calculadas.
float velocityI = 0;                 //Velocidad en [Km/h]
float velocityD = 0;                 //Velocidad en [Km/h]
unsigned long timeold = 0;  // Tiempo transcurrido
const int wheel_diameter = 64;   // Diámetro de la rueda pequeña[mm]
static volatile unsigned long debounceI = 0; // Tiempo del rebote.
static volatile unsigned long debounceD = 0; // Tiempo del rebote.

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
        rpmI = ((float)pulsesI / (float)SLOTS_ENCODER ) * 60 * 4 ; // Calculamos las revoluciones por minuto
        rpmD = (60 * 1000 / SLOTS_ENCODER )/ (millis() - timeold)* pulsesI; // Calculamos las revoluciones por minuto
        velocityI = rpmI * PI * WHEEL_DIAMETER * 60 / 1000000; // Cálculo de la velocidad en [Km/h] 
        velocityD = rpmI * PI * WHEEL_DIAMETER * 60 / 1000000; // Cálculo de la velocidad en [Km/h] 
        timeold = millis(); // Almacenamos el tiempo actual.
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
        Serial.print(pulsesI);
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
            pulsesI++;
        }
    }
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
            pulsesD++;
        }
    }
}

/**
 * Genera la trayectoria deseada
*/
double* trayectoria(double time){
    // Constantes
    double T = 10;
    int r = 1; // Radio del circulo
    double f = 1/T;
    double *resultado; //Apuntador para generar la salida

    double thetad = 2*PI*f*time;
    double thetadp = 2*PI*f;

    double xd = r*sin(thetad);
    double yd = -r*cos(thetad);

    double xdp = r*thetadp*cos(thetad);
    double ydp = r*thetadp*sin(thetad);

    // Generamos el arreglo de salida
    resultado[0] = xd;
    resultado[1] = yd;
    resultado[2] = thetad;
    resultado[3] = xdp;
    resultado[4] = ydp;
    resultado[5] = thetadp;
    return resultado;
}

double* controlYPlanta(double *entradas){
    double Xd = entradas[0];
    double Yd = entradas[0];
    double thetad = entradas[0];
    double Xdp = entradas[0];
    double Ydp = entradas[0];
    double thetadp = entradas[0];
    double X = entradas[0];
    double Y = entradas[0];
    double theta = entradas[0];

    double k1 = 20;
    double k2 = 20;
    double d = 0.01f;
    double *resultado;

    double V1aux = Xdp + k1*(Xd - X);
    double V2aux = Ydp + k2*(Yd - Y);

    double V = V1aux*cos(thetad) + V2aux*sin(thetad);
    double W = -V1aux*sin(thetad)/d + V2aux*cos(thetad)/d;
    resultado[0] = V;
    resultado[1] = W;
}

double* mapeo(double *vectorLlantas){
    int r = 1;
    double L = 1.5;
    double V = vectorLlantas[0];
    double W = vectorLlantas[1];
    double *resultado;

    // double T[2][2] = {{r/2, r/2}, {r/(2*L), r/(2*L)}} //r/2*[1 1; 1/L -1/L];
    double Tinv[2][2] = {{1/r, 1/(r*L)}, {1/r, -1/(r*L)}};

    double wd = Tinv[0][0]*V + Tinv[0][1]*W;
    double wi = Tinv[1][0]*V + Tinv[1][1]*W;
    resultado[0] = wd;
    resultado[1] = wi;
    
    return resultado;
}

double* salida(double *wheels, double theta){
    int r = 1;
    double L = 1.5;
    double wd = wheels[0];
    double wi = wheels[1];
    double *salida;


    double xp = (r/2)*(wd+wi)*cos(theta);
    double yp = (r/2)*(wd+wi)*sin(theta);
    double thetap = (r/2*L)*(wd-wi);

    salida[0] = xp;
    salida[1] = yp;
    salida[2] = thetap;
}

