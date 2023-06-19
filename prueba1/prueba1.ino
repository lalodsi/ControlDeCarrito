int tics = 0;

void setup()
{
    pinMode(2, INPUT_PULLUP);
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(2), counterI, FALLING); // Configuración de la interrupción 0, donde esta conectado. 
    
}

void loop()
{
    Serial.println(digitalRead(2));

}

void counterI(){
    // debounceI = micros();
    if(
        digitalRead(2) &&
        (micros() > 100)
    )
    {
        // Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
        // debounceI = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
        // pulsesI++;
    }
}