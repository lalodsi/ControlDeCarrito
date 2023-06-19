
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
    if (millis() - timeold >= 250){  // Se actualiza cada segundo
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
        Serial.print(rpmI);
        Serial.print(",");
        //Serial.println(velocityI);
        pulsesI = 0;  // Inicializamos los pulsos.
        pulsesD = 0;  // Inicializamos los pulsos.
      interrupts(); // Restart the interrupt processing // Reiniciamos la interrupción
   }
}

void counterI(){
    debounceI = micros();
    if(
        digitalRead(PIN_ENCODER_I)
        // (debounceI > 100)
    )
    {
        // Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
        // debounceI = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
        pulsesI++;
    }
}
void counterD(){
    if(
        digitalRead(PIN_ENCODER_I) &&
        (debounceD > 500)
    )
    {
        // Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
        debounceD = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
        pulsesD++;
    }
}