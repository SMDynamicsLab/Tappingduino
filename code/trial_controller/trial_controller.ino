//Un timer controla dos pines de salida
//Estímulo: seno -pin 11 (OC1A)- Van Vugt script adaptation. pin 12: OC1B 
//Feedback: un timer controla dos pines:  seno -pin 5 (OC3A) y pin2 (OC3B) - Van Vugt script adaptation. pin 3: OC3C, pin 2: OC3B 
//trial eterno (sin Matlab)
//Manda estímulo al pin 11 y al pin 12 (timer 1)
//y feedback al pin 5 y al pin 2 (timer 3)

#define stimPinR 12   //ESTÍMULO EN PIN 12 ENVIADO POR EL TIMER 1 (En función initTimers).
#define stimPinL 11   //ESTÍMULO EN PIN 11 ENVIADO POR EL TIMER 1 (En función initTimers).
#define fdbkPinR 2    //FEEDBACK EN PIN 2 ENVIADO POR EL TIMER 3 (En función initTimers).
#define fdbkPinL 5    //FEEDBACK EN PIN 2 ENVIADO POR EL TIMER 3 (En función initTimers).
#define noisePinR 9   //RUIDO BLANCO (Comparte pin con noisePinL)
#define noisePinL 9   //RUIDO BLANCO (Comparte pin con noisePinR)
#define pinVG A11     //VALOR DE 0V VIRTUAL GENERADO POR EL SHIELD PARA LA SENOIDAL. 

#define LFSR_INIT  0xfeedfaceUL   //ES UN VALOR TOMADO DE INTERNET PARA LA RUTINA DE RUIDO BLANCO.
#define LFSR_MASK  ((unsigned long)( 1UL<<31 | 1UL <<15 | 1UL <<2 | 1UL <<1  ))   //SE UTILIZA TAMBIÉN PARA LA RUTINA DE RUIDO BLANCO.

//http://sphinx.mythic-beasts.com/~markt/ATmega-timers.html
boolean stimRight=true;  //pin12 - timer 1, channel B: right. TRUE: CARGA UN VALOR DE LA TABLA DEL SENO EN EL REGISTRO DE COMPARACIÓN DEL TIMER1 OCR1B
boolean stimLeft=true;  //pin11 - timer 1, channel A: left.   TRUE: CARGA UN VALOR DE LA TABLA DEL SENO EN EL REGISTRO DE COMPARACIÓN DEL TIMER1 OCR1A
boolean fdbkRight=true; //pin2 - timer 3, channel B: right.   TRUE: CARGA UN VALOR DE LA TABLA DEL SENO EN EL REGISTRO DE COMPARACIÓN DEL TIMER1 OCR3B
boolean fdbkLeft=true;  //pin5 - timer 3, channel A: left.    TRUE: CARGA UN VALOR DE LA TABLA DEL SENO EN EL REGISTRO DE COMPARACIÓN DEL TIMER1 OCR3A
boolean noise = false;                                      //SE UTILIZA EN LA INTERRUP DEL TIMER 2 PARA LA GENERACIÓN DEL RUIDO BLANCO

unsigned long int t=0;
long int prevStim_t=0;  //UTILIZADO EN LOOP() PARA EL CÁLCULO DE TIEMPO PARA EL ENVÍO DEL ESTÍMULO.
long int prevFdbk_t=0;  //UTILIZADO EN LA FUNCIÓN GET_PARAMETERS() EN EL CÁLCULO DE TIEMPO PARA LA RECEPCIÓN DE RESPUESTA POR PULSADOR
boolean stim_flag=false;//TRUE: LA VARIABLE AUX1 TOMA EL VALOR DE LA TABLA DE LA SENOIDAL
boolean perturb_flag1=false;//UTILIZADO EN PHASE SHIFT PERTURBACIÓN
boolean perturb_flag2=false;//UTILIZADO EN PHASE SHIFT PERTURBACIÓN
boolean fdbk_flag=false;//TRUE: LA VARIABLE AUX3 TOMA EL VALOR DE LA TABLA DE LA SENOIDAL

unsigned int stimFreq = 440;//(C6) // defines the frequency (i.e., pitch) of the tone (in Hz)
unsigned int fdbkFreq = 660;//(C6) // defines the frequency (i.e., pitch) of the tone (in Hz)

int vg = 0; //VALOR EN ENTERO DE LA MEDICIÓN DEL 0V VIRTUAL ENTREGADO POR EL SHIELD, ESCALADO A 10BIT (1024)
int read_vg = 0;  //TOMA SU VALOR DE LA FUNCIÓN READVIRTUALGROUND()
int amplitude = 120 ;//125;//1;  //SE USA EN VARIOS LADOS. TODAVÍA NO SÉ PARA QUE ES?????????????????????
int cycle_counter=0;
int num_cycles=3;
unsigned int max_num_cycles = 4,  min_num_cycles = 2; //(variable global)

int transitorio = 5;
int sensorIzqPin=A9;    //sensor de la izquierda
int voltajeSensorIzq;
//int voltajeSensorDer;
unsigned int NStim;
unsigned int stimNumber = 1;
unsigned int respNumber = 1;
//unsigned long t;
//unsigned long prevStim_t = 0;
unsigned long prevResp_t = 0;
unsigned long perturbBip_t;
unsigned int perturbBip;
unsigned int perturbationDone;
unsigned int perturbSize;
unsigned voltajeSampleCounter = 0;
unsigned long prevVoltajeSample_t = 0;
unsigned int *voltajeSensor;

int sensorThreshold = 50;
boolean voltajeSensorIzq_flag = false;
//boolean voltajeSensorDer_flag = false;
boolean readVoltajeSensorOn = false; // true;
boolean dosMuestrasXmili_flag = false;
//Duración del Tap. La cantidad total de milisegundos en los que el sensor registra la fuerza es: "tiempo1000HZ" + "tiempo500HZ"
int tiempo1000HZ=100; //milisegundos luego del tap en los que el sensor registra con una frecuencia de sampleo de 1KHz 
int tiempo500HZ=160; //milisegundos luego del finalizado tiempo1000 en los que el sensor registra con una frecuencia de sampleo de 500 Hz  


////Store data in memory
char *eventName;
unsigned int *eventNumber;
unsigned long *eventTime;
unsigned int eventCounter = 0;

//

uint16_t phaseIncrementStim = 0;  // 16 bit delta     //ESTOS CUATRO SE USAN PARA MODIFICAR EL INDEX DEL ARREGLO DE LA SEÑAL SENOIDAL, PERO NO ENTIENDO COMO FUNCIONA. HACE UN CORRIMIENTO????
uint16_t phaseIncrementFdbk = 0; // 16 bit delta
uint16_t phaseAccumulatorStim = 0;  // 16 bit accumulator
uint16_t phaseAccumulatorFdbk = 0; // 16 bit accumulator
int stim_counter; //VA AUMENTANDO EN LA INTERRUPCIÓN DEL TIMER1 Y SE COMPARA CON STIM_DURATION_CYCLES (DURACIÓN DEL ESTÍMULO). CUANDO TERMINA EL CICLO SE RESETEA.
int fdbk_counter; //VA AUMENTANDO EN LA INTERRUPCIÓN DEL TIMER3 Y SE COMPARA CON FDBK_DURATION_CYCLES (DURACIÓN DEL ESTÍMULO). CUANDO TERMINA EL CICLO SE RESETEA.

// DDS resolution
const uint32_t freq_samp = 31180; // 16MHz/513 porque ahora el pin está en modo 9 bits
const uint16_t resolution_DDS = pow(2,16)/freq_samp;


//AGREGO
uint16_t FDBK_DURATION_CYCLES = 23; // feedback duration (cycles).  //CICLOS DE DURACIÓN DEL CICLO
uint16_t STIM_DURATION_CYCLES = 23; // stimulus duration (cycles)   //CICLOS DE DURACIÓN DEL ESTÍMULO
#define STIM_DURATION 50 //stimulus duration (milliseconds)         //NO ES UTILIZADO
#define FDBK_DURATION 50 //este lo agrego yo                        //NO ES UTILIZADO
#define ANTIBOUNCE (0.5*isi)//minimum interval between responses (milliseconds)
boolean allow;  //CUANDO ALLOW ES FALSE, SE REGISTRA EL EVENTO DEL TRIAL
int fdbk;       //REGISTRA EL VALOR DE ENTRADA DEL PIN QUE LEE LA RESPUESTA DEL SUJETO DE ESTUDIO
int i;
unsigned int stim_number;     //REGISTRA EL NÚMERO DE ESTÍMULO
unsigned int fdbk_number;     //REGISTRA EL NÚMERO DE FEEDBACK
unsigned int *event_number;   //ENTIENDO QUE ES UN PUNTERO PARA RECORRER UN ARREGLO CON EL NÚMERO DE EVENTO
char *event_name;             //ENTIENDO QUE ES UN PUNTERO PARA RECORRER UN ARREGLO CON EL NOMBRE DEL EVENTO.
unsigned long *event_time;    //ENTIENDO QUE ES UN PUNTERO PARA RECORRER UN ARREGLO CON LOS TIEMPOS DE CADA EVENTO.
unsigned int event_counter;   //ENTIENDO QUE ES UN ÍNDICE QUE SE UTILIZA PARA RECORRER LOS ESPACIOS DE MEMORIA INDICADOS POR LOS TRES PUNTEROS ANTERIORES.

unsigned int isi=500;   //INICIALIZACIÓN DEL PERÍODO DEL ESTÍMULO, EL VALOR UTILIZADO SE RECIBE POR PYTHON.
unsigned int prevISI=0; //AUXILIAR PARA GUARDAR PERÍODO, EN EXPERIMENTO DE PERTURBACIÓN CON PHASE SHIFT.
unsigned int n_stim=3;  //Entiendo que son valores iniciales por las dudas //NO ENTIENDO BIEN PARA QUE SE USA ESTA VARIABLE, PARECIERA QUE PARA INDICAR CUÁL VA A SER EL NÚMERO TOTAL DE ESTÍMULOS
int perturb_size=0;     //TIEMPO DE PERTURBACIÓN QUE MODIFICA AL ISI(PERÍODO). ES UN INT, PORQUE EL DATO VIENE DE PYTHON Y PUEDE TENER SIGNO. 
unsigned int perturb_bip=0;   //SE ENVÍA VALOR DESDE PYTHON PARA INDICAR EN QUE BEEP SE VA A REALIZAR UNA PERTURBACIÓN EN EL ESTÍMULO.
unsigned int perturb_type=0;  //SELECCIONA EL TIPO DE PERTURBACIÓN 0->S(STEP CHANGE), 1->(PHASE SHIFT).
#define INPUTPIN A9   //DEFINICIÓN DEL PIN DE ENTRADA DE RESPUESTA DEL USUARIO.
char message[20];     //NO ENTIENDO SI SE ESTÁ USANDO, NO LO VEO PUESTO EN PROGRAMA EN FORMATO ARREGLO.  
boolean SR=false, SL=false, FR=false, FL=false, NR=false, NL=false; //ESTÁN DENTRO DE UN SWITCH CASE, TODAVÍA NO ENTIENDO PARA QUE SE USAN.

//////////// Set up lookup table for waveform generation
// sine wavefunction
static const uint8_t  sineTable[] PROGMEM = {   //UTILIZA PROGMEM PARA QUE LA TABLA SE GUARDE EN LA MEMORIA FLASH EN VEZ DE EN LA SRAM (ESTA ÚLTIMA ES VOLÁTIL)
0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,
0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae,
0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,
0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,
0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,
0xea,0xec,0xed,0xef,0xf0,0xf2,0xf3,0xf5,
0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfc,
0xfd,0xfe,0xfe,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0xfe,
0xfd,0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,
0xf6,0xf5,0xf3,0xf2,0xf0,0xef,0xed,0xec,
0xea,0xe8,0xe6,0xe4,0xe2,0xe0,0xde,0xdc,
0xda,0xd8,0xd5,0xd3,0xd1,0xce,0xcc,0xc9,
0xc7,0xc4,0xc1,0xbf,0xbc,0xb9,0xb6,0xb3,
0xb0,0xae,0xab,0xa8,0xa5,0xa2,0x9f,0x9c,
0x98,0x95,0x92,0x8f,0x8c,0x89,0x86,0x83,
0x80,0x7c,0x79,0x76,0x73,0x70,0x6d,0x6a,
0x67,0x63,0x60,0x5d,0x5a,0x57,0x54,0x51,
0x4f,0x4c,0x49,0x46,0x43,0x40,0x3e,0x3b,
0x38,0x36,0x33,0x31,0x2e,0x2c,0x2a,0x27,
0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,
0x15,0x13,0x12,0x10,0x0f,0x0d,0x0c,0x0a,
0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x03,
0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
0x02,0x03,0x03,0x04,0x05,0x06,0x07,0x08,
0x09,0x0a,0x0c,0x0d,0x0f,0x10,0x12,0x13,
0x15,0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,
0x25,0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,
0x38,0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,
0x4f,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,
0x67,0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c
};

// ---- Hasta aca definiciones, etc ------------------------------------------------------------------------------


ISR(TIMER1_OVF_vect) {
  // wavetable lookup index (upper 8 bits of the accumulator)   //indice de búsqueda de tabla de ondas (8 bits superiores del acumulador)
  uint8_t index1 = 0;   //ÍNDICE PARA EL RECORRIDO DE LA TABLA DEL SENO EN LA MEMORIA FLASH
  uint16_t aux1;        //AUXILIAR PARA GUARDAR EL VALOR PWM DE LA TABLA SENO QUE SE RECUPERA DE LA MEMORIA FLASH
  
  // Update accumulator
  phaseAccumulatorStim += phaseIncrementStim;
  index1 = phaseAccumulatorStim >> 8;                     //GUARDA EN INDEX1 EL EL BYTE MÁS SIGNIFICATIVO DE phaseAccumulatorStim.
  if(index1 <= (phaseIncrementStim >>8))  stim_counter++; // counter aumenta cuando index1 resetea

  OCR1A = vg;   //CARGAN EN EL REGISTRO OCR1A EL VALOR EN ENTERO DE LA MEDICIÓN DEL 0V VIRTUAL ENTREGADO POR EL SHIELD, ESCALADO A 10BIT (1024)?? ES PARA DETECTAR EL CRUCE POR CERO?? LA CANTIDAD DE BITS DE LOS REGISTROS NO COINCIDEN.
  OCR1B = vg;   //CARGAN EN EL REGISTRO OCR1B EL VALOR EN ENTERO DE LA MEDICIÓN DEL 0V VIRTUAL ENTREGADO POR EL SHIELD, ESCALADO A 10BIT (1024)?? ES PARA DETECTAR EL CRUCE POR CERO?? LA CANTIDAD DE BITS DE LOS REGISTROS NO COINCIDEN.

    
// if (t-prevStim_t<STIM_DURATION){
  if (stim_counter< STIM_DURATION_CYCLES){ // VERIFICA SI stim_counter ES MENOR A LA DURACIÓN DEL ESTÍMULO.
  
   if(stim_flag==true){                             //HABILITA QUE LA VARIABLE AUX1 TOME EL VALOR DE LA TABLA DE LA SENOIDAL
    aux1 = pgm_read_byte(&sineTable[index1]);       //ESTA FUNCIÓN SE UTILIZA PARA LEER UN BYTE UBICADO EN LA MEMORIA FLASH (EN ESTE CASO LE PASA LA DIRECCIÓN DE MEMORIA ADONDE ESTÁ APUNTANDO EL ÍNDICE)
  // Read oscillator value for next interrupt
    
    if (stimLeft==true)  OCR1A = aux1;    //CARGA EL VALOR ACTUAL DEL PWM PARA LA CONSTRUCCIÓN DE LA SENOIDAL EN EL REGISTRO DE COMPARACIÓN OCR1A

    if (stimRight==true)   OCR1B = aux1;  //CARGA EL VALOR ACTUAL DEL PWM PARA LA CONSTRUCCIÓN DE LA SENOIDAL EN EL REGISTRO DE COMPARACIÓN OCR1B
  
  }  
 }

else  stim_flag=false;    //SI stim_counter ES >= A LA DURACIÓN DEL ESTÍMULO, stim_flag = FALSE Y YA NO SE MODIFICAN LOS REGISTROS OCR1A Y OCR1B

}


ISR(TIMER3_OVF_vect) {
  // wavetable lookup index (upper 8 bits of the accumulator)
  uint8_t index3 = 0;   //ÍNDICE PARA EL RECORRIDO DE LA TABLA DEL SENO EN LA MEMORIA FLASH
  uint16_t aux3;        //AUXILIAR PARA GUARDAR EL VALOR PWM DE LA TABLA SENO QUE SE RECUPERA DE LA MEMORIA FLASH
  
  // Update accumulator
  phaseAccumulatorFdbk += phaseIncrementFdbk;
  index3 = phaseAccumulatorFdbk >> 8;                     //GUARDA EN INDEX3 EL EL BYTE MÁS SIGNIFICATIVO DE phaseAccumulatorFdbk.
  if(index3 <= (phaseIncrementFdbk >>8))  fdbk_counter++; // counter aumenta cuando index3 resetea

  OCR3A = vg; //CARGAN EN EL REGISTRO OCR3A EL VALOR EN ENTERO DE LA MEDICIÓN DEL 0V VIRTUAL ENTREGADO POR EL SHIELD, ESCALADO A 10BIT (1024)?? ES PARA DETECTAR EL CRUCE POR CERO?? LA CANTIDAD DE BITS DE LOS REGISTROS NO COINCIDEN.
  OCR3B = vg; //CARGAN EN EL REGISTRO OCR3B EL VALOR EN ENTERO DE LA MEDICIÓN DEL 0V VIRTUAL ENTREGADO POR EL SHIELD, ESCALADO A 10BIT (1024)?? ES PARA DETECTAR EL CRUCE POR CERO?? LA CANTIDAD DE BITS DE LOS REGISTROS NO COINCIDEN.

//  if (t-prevFdbk_t<FDBK_DURATION){
  if (fdbk_counter< FDBK_DURATION_CYCLES){        // VERIFICA SI fdbk_counter ES MENOR A LA DURACIÓN DEL ESTÍMULO.
    
    if(fdbk_flag==true){                          //HABILITA QUE LA VARIABLE AUX3 TOME EL VALOR DE LA TABLA DE LA SENOIDAL
      aux3 = pgm_read_byte(&sineTable[index3]);   //ESTA FUNCIÓN SE UTILIZA PARA LEER UN BYTE UBICADO EN LA MEMORIA FLASH (EN ESTE CASO LE PASA LA DIRECCIÓN DE MEMORIA ADONDE ESTÁ APUNTANDO EL ÍNDICE)
    // Read oscillator value for next interrupt
   
      if (fdbkLeft==true)  OCR3A = aux3;          //CARGA EL VALOR ACTUAL DEL PWM PARA LA CONSTRUCCIÓN DE LA SENOIDAL EN EL REGISTRO DE COMPARACIÓN OCR3A
  
      if (fdbkRight==true)   OCR3B = aux3;        //CARGA EL VALOR ACTUAL DEL PWM PARA LA CONSTRUCCIÓN DE LA SENOIDAL EN EL REGISTRO DE COMPARACIÓN OCR3B
    
    }  
   }
  
  else  fdbk_flag=false;                          //SI fdbk_counter ES >= A LA DURACIÓN DEL ESTÍMULO,fdbk_flag = FALSE Y YA NO SE MODIFICAN LOS REGISTROS OCR3A Y OCR3B
  
}


ISR(TIMER2_OVF_vect){   //TIMER PARA RUTINA DE RUIDO BLANCO
  OCR2A = vg;

  if(noise==true){
    if(cycle_counter==0)
      // setea OCR arriba durante la primera mitad del período de ruido
      OCR2B = vg+amplitude;
    if(cycle_counter==round(num_cycles/2.0))
      // setea OCR abajo durante la segunda mitad del período de ruido
      OCR2B = vg-amplitude;
    if(cycle_counter==num_cycles-1){
      // vuelve a empezar, nuevo período de ruido
      num_cycles = generateNoise();
      cycle_counter = -1;
    }
    cycle_counter++;
  }
//  Serial.println(cycle_counter);
}
//máximo número de ciclos consecutivos del timer

// --------- Primer Void ---------------------------------------------

void initTimers(void){                                //FUNCIÓN PARA INICIALIZAR TIMERS
  // Stimulus
  // Set pins as output
  pinMode(stimPinR,OUTPUT);                           //CONFIGURACIÓN DEL PIN 12 COMO SALIDA DEL ESTÍMULO AL LADO DERECHO
  pinMode(stimPinL,OUTPUT);                           //CONFIGURACIÓN DEL PIN 11 COMO SALIDA DEL ESTÍMULO AL LADO DERECHO

  // 9-bit Fast PWM - non inverted PWM
  TCCR1A= _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);     

  // Start timer without prescaler
  TCCR1B = _BV(WGM12) | _BV(CS10);
  //TCCR1B = _BV(WGM12) | _BV(CS11);  //PRESCALER = 8
  // Enable overflow interrupt for OCR1A
  TIMSK1 = _BV(TOIE1);

  // Feedback
  // Set pins as output
  pinMode(fdbkPinR,OUTPUT);
  pinMode(fdbkPinL,OUTPUT);

  // 9-bit Fast PWM - non inverted PWM
  TCCR3A= _BV(COM3A1) | _BV(COM3B1) | _BV(WGM31);
  // Start timer without prescaler
  //  TCCR3B = _BV(WGM32) | _BV(CS32);
  TCCR3B = _BV(WGM32) | _BV(CS30);
  //TCCR3B = _BV(WGM32) | _BV(CS31);  //PRESCALER = 8
  // Enable overflow interrupt for OCR1A and OCR1B
  TIMSK3 = _BV(TOIE3);


  //Noise
  // Set pins as output
  pinMode(noisePinR,OUTPUT);
  pinMode(noisePinL,OUTPUT);

  TIMSK2 = _BV(TOIE2); 
  //Dos Registros del timer2
  TCCR2A  = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  //TCCR2B  = _BV(CS20) | _BV(WGM22);   //NO PRESCALER
  TCCR2B  = _BV(CS21);// | _BV(WGM22);   //PRESCALER EN 8
  //TCCR2B  = _BV(CS21) | _BV(CS20) | _BV(WGM22);   //PRESCALER EN 32 
  //TCCR2B  = _BV(CS22) | _BV(WGM22);   //PRESCALER EN 64 
  //TCCR2B  = _BV(CS22) | _BV(CS20) | _BV(WGM22);   //PRESCALER EN 128 
  // Fast PWM 8bits non inverted, CTC, TOF2 on TOP (OCR2A)
}


// Función que genera el ruido blanco
unsigned int generateNoise(void){
   unsigned int num_cycles;
  //  uint8_t aux=0;
   static unsigned long int lfsr = LFSR_INIT; // See https://en.wikipedia.org/wiki/Linear_feedback_shift_register#Galois_LFSRs
   // 32 bit init, nonzero 
   // If the output bit is 1, apply toggle mask. The value has 1 at bits corresponding to taps, 0 elsewhere. 
   if(lfsr & 1) {
    lfsr = (lfsr >>1) ^ LFSR_MASK; 
    }
   else{
    lfsr >>= 1;
    }
   num_cycles = min_num_cycles + (lfsr % (max_num_cycles-min_num_cycles)); // correción: como mínimo 2  
  return(num_cycles);
}


// Lectura de la tierra virtual desde un pin analógico
int readVirtualGround(void)
{
  return analogRead(pinVG);
}


/* Translates the desired output frequency to a phase increment to be used with the phase accumulator.*/
uint16_t setFrequency( uint16_t frequency ){
  uint16_t phaseIncr16 =  resolution_DDS * frequency;
  return phaseIncr16;//(phaseIncr32 >> 16);
}

uint16_t setCycles( uint16_t frequency ){
  return 0.05*frequency;
}

// Function to start playing sound
void SoundSwitch(char tipo,uint16_t freq, boolean left, boolean right){
/* This function enables the timerOverflowInterrupt in the time mask depending on tipo: 
if tipo is 's', selects Timer 1 (stimulus)
if tipo is 'f' selects Timer 3 (feedback)
if tipo is 'n' selects Timer 2 (Noise)
the left and right bools as true or false selects the channels where the signals will come out
for 's' and 'f', freq is the frequency of the beep.
for 'n', freq is the amplitud in GenerateNoise, meaning the volume of the white noise.
*/
  switch(tipo){
    case 's':
      phaseAccumulatorStim = 0;
      phaseIncrementStim = setFrequency(freq);
      stim_counter = 0;
      STIM_DURATION_CYCLES = setCycles(freq);
      stimRight = right;
      stimLeft = left;      
      break;
      
  
    case 'f':
      phaseAccumulatorFdbk = 0;
      phaseIncrementFdbk = setFrequency(freq);
      fdbk_counter = 0;
      FDBK_DURATION_CYCLES = setCycles(freq);
      fdbkRight = right;
      fdbkLeft = left;
      break;

  
    case 'n':
      amplitude = freq;
      if(right && left ==1)
        noise = true;
      else noise = false;
      break;
  }  
}


//---------------------------------------------------------------------
//print a line avoiding "carriage return" of Serial.println()
void serial_print_string(char *string) {
  Serial.print(string);
  Serial.print("\n");
  return;
}



//---------------------------------------------------------------------
//parse data from serial input
//input data format: eg "I500;n30;P-10;B15;T0;SR;FL;NB;A3;X"    //I500(Período del estímulo - isi = 500); N30(Número de estímulos - n_stim = 30); P-10(Tamaño de perturbación - perturb_size = -10);
void parse_data(char *line) {                                   //B15(Bip de perturbación (15-20) - perturb_bip = 15); T0(Tipo de perturbación - perturb_type = 0); SR(Estímulo por oído derecho - SR)
  char field[10];                                               //FL(Feedback por oído izquierdo - FL); NB(Ruido blanco por ambos oídos - NB); A3(Amplitud - amplitude = 3); 
  int n,data;                                                   //Desde Python: IOk, nOk, P(No usado), B(No usado), T(No usado), SOk, FOk, NOk, AOk. 
  //scan input until next ';' (field separator)
  while (sscanf(line,"%[^;]%n",field,&n) == 1) {
    data = atoi(field+1);
    //parse data according to field header
    switch (field[0]) {
      case 'I':
        isi = data;
        break;
      case 'n':
        n_stim = data;
        break;
      case 'P':
        perturb_size = data;
        break;
      case 'B':
        perturb_bip = data;
        break;
        
      case 'T':                     //SE CAMBIO E POR T. ADS 05/22
        sensorThreshold = data;
        break;         
        
      case 'S':
        switch (field[1]){
          case 'R':
            SR = true;
            SL = false;
            break;
          case 'L':
            SR = false;
            SL = true;
            break;
          case 'B':
            SR = true;
            SL = true;
            break;
          case 'N':
            SR = false;
            SL = false;
            break;
        }
        break;
      case 'F':
        switch (field[1]){
          case 'R':
            FR = true;
            FL = false;
            break;
          case 'L':
            FR = false;
            FL = true;
            break;
          case 'B':
            FR = true;
            FL = true;
            break;
          case 'N':
            FR = false;
            FL = false;
            break;
        }
        break;

       case 'N':
        switch (field[1]){
          case 'B':
            NR = true;
            NL = true;
            break;
          case 'N':
            NR = false;
            NL = false;
            break;
        }
       
       case 'A':
        amplitude = data;
        break; 

      default:
        break;
    }
    line += n+1;
//    if (*line != ';')
  //    break;
//    while (*line == ';')
//      line++;
  }
  return;
}


//---------------------------------------------------------------------

//---------------------------------------------------------------------

void get_parameters() {
  char line[45];
  char i,aux='0';
  i = 0;
  
  //directly read next available character from buffer
  //if flow ever gets here, then next available character should be 'I'
//  aux = Serial.read();

  //Turn on noise
  //SoundSwitch('n',amplitude,true,true);     //ENVÍA SONIDO BLANCO A AMBOS OÍDOS, Queda prendido durante todo el experimento
//  SoundSwitch('n',amplitude,false,false);     //ENVÍA SONIDO BLANCO A AMBOS OÍDOS
//SoundSwitch('n',amplitude,NL,NR);

  while (Serial.available() < 1) {
      t = millis();
      //allow user to tap while waiting for data from computer
      if ((t - prevFdbk_t) > ANTIBOUNCE && fdbk_flag==false) {        
        fdbk = digitalRead(INPUTPIN);
        if (fdbk == HIGH){
          SoundSwitch('f', fdbkFreq, true, true);
          prevFdbk_t=t;
          fdbk_flag=true;
        }
      } 
  }
  aux = Serial.read();

  //read buffer until getting an X (end of message)
  while (aux != 'X') {
  //keep reading if input buffer is empty
    while (Serial.available() < 1) {}
    line[i] = aux;
    i++;
    aux = Serial.read();
  }
  line[i] = '\0';         //terminate the string

  //just in case, clear incoming buffer once read
  //Serial.flush();
  while(Serial.read()>=0);
  //parse input chain into parameters
  parse_data(line);
  return;

}
//---------------------------------------------------------------------
void save_data(char ename, unsigned int enumber, unsigned long etime){
      //store event data
      event_name[event_counter] = ename;
      event_number[event_counter] = enumber;
      event_time[event_counter] = etime;
      event_counter++;

      switch(ename){
        case 'S':
          stim_number++;
          break;

        case 'R':
          fdbk_number++;
          break;
      }    
}


void setup() {

  Serial.begin(57600); //USB communication with computer / 9600 57600 115200
  pinMode(INPUTPIN,INPUT);
  
  cli();
  
  initTimers();

    //Turn on noise
  //SoundSwitch('n',120,true,true);     //ENVÍA SONIDO BLANCO A AMBOS OÍDOS
  
  // Unica lectura de tierra virtual hasta que el loop sea el correcto (condicion para iniciar)
  read_vg = readVirtualGround();
  vg = (int) (read_vg*256/1024);

  allow = false;

  sei();
}



void loop() {

  if(allow == false){   //ALLOW = FALSE, PERMITE QUE SE LEA UN MENSAJE DESDE PYTHON
      
    //just in case, clear incoming buffer once read
    Serial.flush();
        
        //Turn on noise
    SoundSwitch('n',120,true,true);     //ENVÍA SONIDO BLANCO A AMBOS OÍDOS

    get_parameters();   //LEE MENSAJE DE PYTHON DESDE EL SERIAL HASTA QUE ENCUENTRA UNA "X". EL ALGORITMO INTERPRETA LA "X" COMO FIN DE MENSAJE DESDE PYTHON. 
    prevISI = isi;      //GUARDA EN VARIABLE AUXILIAR prevISI EL PERÍODO ORIGINAL isi 
    allow = true;       //PONE EL FLAG ALLOW EN TRUE, PARA QUE NO SE LEA UN NUEVO DATO HASTA QUE NO SE PROCESE EL ACTUAL.

    stim_number = 0;    //INICIALIZA VARIABLE
    fdbk_number = 0;    //INICIALIZA VARIABLE
    event_counter = 0;  //INICIALIZA VARIABLE
    perturb_flag1 = false;
    perturb_flag2 = false;
    prevVoltajeSample_t = 0;
    voltajeSensorIzq_flag = false;
    voltajeSampleCounter = 0;

    event_name = (char*) calloc(3*n_stim,sizeof(char));                     //RESERVA UNA MEMORIA PARA 3*n_stim ELEMENTOS DE TAMAÑO sizeof(char)(CHAR = 1 BYTE). INICIALIZA EL BLOQUE DE MEMORIA EN 0. DEVUELVE UN PUNTERO AL PRIMER CHAR EN event_name. 
    event_number = (unsigned int*) calloc(3*n_stim,sizeof(unsigned int));   //RESERVA UNA MEMORIA PARA 3*n_stim ELEMENTOS DE TAMAÑO sizeof(unsigned int)(unsigned int = 4 BYTEs). INICIALIZA EL BLOQUE DE MEMORIA EN 0. DEVUELVE UN PUNTERO AL PRIMER UNSIGNED INT EN event_number.
    event_time = (unsigned long*) calloc(3*n_stim,sizeof(unsigned long));   //RESERVA UNA MEMORIA PARA 3*n_stim ELEMENTOS DE TAMAÑO sizeof(unsigned long)(unsigned long = 4 BYTEs). INICIALIZA EL BLOQUE DE MEMORIA EN 0. DEVUELVE UN PUNTERO AL PRIMER UNSIGNED LONG EN event_time.
    
    voltajeSensor = (unsigned int*) calloc(n_stim*tiempo1000HZ, sizeof(unsigned int));  //   taps seguro, quizás más
}
else{               //ALLOW = TRUE, CORRE EL TRIAL ENVIADO EN EL MENSAJE DE PYTHON
    t = millis();     //REGISTRA EN t EL TIEMPO ACTUAL

    //Turn on noise
    //SoundSwitch('n',amplitude,NL,NR);     //ENVÍA SONIDO BLANCO A AMBOS OÍDOS

    //Perturbation
    if (stim_number == perturb_bip){
      if ((perturb_type == 0 or perturb_type == 1) and perturb_flag1 == false){
        prevISI = isi;
        isi += perturb_size;
        perturb_flag1 = true;
      }
    }
    if (stim_number == perturb_bip + 1){
      if (perturb_type==1 and perturb_flag2==false) {
        isi = prevISI;
        perturb_flag2 = true;
      }
    }
    
    //Send stimulus 
    if ((t-prevStim_t)>= isi && stim_flag==false) { //enciende el sonido //SI PASÓ UN LAPSO DE TIEMPO MAYOR AL PERÍODO ENTRE ESTÍMULOS Y stim_flag == false
      SoundSwitch('s', stimFreq, SL, SR);                               //ENVÍA ESTÍMULO
      prevStim_t=t;                                                     //REGISTRA EL TIEMPO ACTUAL EN prevStim, PARA VOLVER A CONTAR EL PERÍODO ENTRE ESTÍMULOS
      stim_flag=true;                                                   //DESHABILITA LA POSIBILIIDAD DE ENVIAR UN NUEVO ESTÍMULO
      save_data('S', stim_number, t);                                   //GUARDA EL NÚMERO DE ESTÍMULO Y EL TIEMPO AL CUÁL OCURRIÓ.
    }
    
    //Read response                                             //ESTE BLOQUE IF, ES PARA ENVIAR LA RESPUESTA AL SUJETO
    if ((t - prevFdbk_t) > ANTIBOUNCE && fdbk_flag==false) {    //SE APLICA UN FILTRO ANTIREBOTE, PARA EVITAR QUE SE REGISTRE MÁS DE UNA RESPUESTA
      fdbk = digitalRead(INPUTPIN);                             //SE LEE LA RESPUESTA DEL SUJETO DESDE EL PIN DEL ARDUINO
      if (fdbk == HIGH){                                        //EVALÚA SI EL PIN ESTÁ EN 1
        SoundSwitch('f', fdbkFreq, FL, FR);                     //ENVÍA SONIDO DE FEEDBACK AL SUJETO
        prevFdbk_t=t;                                           //REGISTRA EL TIEMPO ACTUAL EN prevFdbk,PARA UTILIZARLO EN EL FILTRO ANTIREBOTE ((t - prevFdbk_t) > ANTIBOUNCE)
        fdbk_flag=true;                                         //DESHABILITA LA POSIBILIIDAD DE REGISTRAR UNA NUEVA RESPUESTA
        save_data('R', fdbk_number, t);  
        voltajeSensorIzq_flag = true;                    //GUARDA EL NÚMERO DE FEEDBACK Y EL TIEMPO AL CUÁL OCURRIÓ.
      }
    }



    /////Read sensor voltaje


    //Sensor. Intervalo de taps en el que la lectura del sensor está habilitada.
 //   if (stimNumber > (transitorio) && (t - prevStim_t) >= (isi / 2)) { //el sensor es habilitado para registrar cuando faltan tapsPrePertToBeRecorded taps para la perturbación
 //     readVoltajeSensorOn = true;
 //   }

    //if (stimNumber > (perturbBip + 14) && (t - prevStim_t) >= (isi / 2)) { //el sensor se deshabilita 14 taps y medio después de la perturbación 
    //if (stimNumber > (perturbBip + 8) && (t - prevStim_t) >= (isi / 2)) { //el sensor se deshabilita 8 taps y medio después de la perturbación. Usar con NStim=23 en Matlab. Registra la fuerza de 16 taps
    //  if (stimNumber > (perturbBip + tapsPosPertToBeRecorded) && (t - prevStim_t) >= (isi / 2)) { //el sensor se deshabilita 12 taps y medio después de la perturbación. Usar con NStim=27 en Matlab. Registra la fuerza de 20 taps 
    //  readVoltajeSensorOn = false;
    //}

    //1kHz sampling

//    if (stimNumber > (transitorio) && (t - prevVoltajeSample_t) >= 1 && voltajeSensorIzq_flag == true && readVoltajeSensorOn == true && dosMuestrasXmili_flag == false) { //almacena el valor de entrada del sensor, un dato por milisegundo
    if (stim_number > (transitorio) && voltajeSensorIzq_flag == true && dosMuestrasXmili_flag == false) { 
      voltajeSensor[voltajeSampleCounter] = analogRead(sensorIzqPin);
      //tiempo[osc_counter]=t;
      voltajeSampleCounter++;
      dosMuestrasXmili_flag = true;
      prevVoltajeSample_t = t;
    }

    if ((t - prevVoltajeSample_t) >= 1) {
      dosMuestrasXmili_flag = false;
    } 

//Stop sampling at 1KHz
//    if ((t - prevResp_t) >= tiempo1000HZ && (voltajeSensorIzq_flag == true) && readVoltajeSensorOn == true && dosMuestrasXmili_flag == false) { //intervalo de tiempo durante el cual el sensor registra cada tap  (50 ms)
    if ((t - prevFdbk_t) >= tiempo1000HZ && (voltajeSensorIzq_flag == true)) { //intervalo de tiempo durante el cual el sensor registra cada tap  (50 ms)
//      voltajeSensor[voltajeSampleCounter] = 1800; //terminador. Cuando cambia la frecuencia de muestreo
      voltajeSampleCounter++;
      voltajeSensorIzq_flag = false;
      //voltajeSensorDer_flag = false;
      dosMuestrasXmili_flag = true;
    }

    ////perturbation
    if (stimNumber == perturbBip && perturbationDone == false) {
      //step change or first perturbation of phase shift
      isi += perturbSize;
      perturbationDone = true;
      
    }



    //End trial
    //allow one more period (without stimulus)
    if ((stim_number >= n_stim) && ((t - prevStim_t) >= (0.5 * isi))) {
      for (i = 0; i<event_counter; i++) {
        sprintf(message,"%c %d %ld",event_name[i],event_number[i],event_time[i]);
        
        serial_print_string(message);
      }
     //  Serial.println("E");  //send END message

      allow = false;
      
      for (i = 0; i < voltajeSampleCounter; i++) { //send sensor data to Matlab
        // sprintf(message,"%d: %ld;",osc[i],tiempo[i]);           //%d decimal integer
        sprintf(message, "V %d %d", i, voltajeSensor[i]);         //%d decimal integer
//        Serial.println(message);
        serial_print_string(message);
      }
      Serial.println("E");  //send END message

      //turn off noise 
      SoundSwitch('s',1,false,false);
      SoundSwitch('f',1,false,false);
      SoundSwitch('n',120,false,false); 

      free(event_name);
      free(event_number);
      free(event_time);
      free(voltajeSensor); 
    }

  }
  
}
