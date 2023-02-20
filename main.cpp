#include "mbed.h"

float velocidad = 10; // dejar siempre en 10 

float Kpn = 0.35; //.1
long Kdn = 65; // 8
long Kde = 65; // 25
long xu = 150 ;  // repeticiones

int reversa = 100;   // velocidad ala que ira el motor hacia atras 
int frente = 200;   // velovidad ala que ira el motor hacia enfrente 
int turbina = 70;   // velocidad de turbina //70
int line_data = 2;   // elegir el color de la linea, 2 negro o 1 blanco 
int modulo_on = 2;  // 0 = solo usar boton. 1 = usar modulo solo para iniciar. 2 = usar modulo para iniciar y apagar 
char allow = 1;  // permitir o no el uso dela EDF manualmente sin interfaz 0 y 1 





int memoria_on = 0;




Timeout interrupt_edf;  
Timeout interrupt_pid;  

Serial pc ( USBTX , USBRX);

DigitalOut uno(PF_1);
DigitalOut dos(PF_0);
DigitalOut tres(PB_1);
DigitalOut cuatro(PB_6);

AnalogIn sensores(PB_0);
DigitalOut led(LED1);
DigitalOut led1(LED1);
DigitalOut led2(LED1);
DigitalOut led3(LED1);
AnalogIn  boton ( PA_4);
//AnalogIn encoder ( PA_3);



PwmOut mdf(PA_8);
PwmOut mda(PA_11);

PwmOut mia(PB_4);
PwmOut mif(PB_5);



DigitalOut brush( PA_9);

Timer timerb; // brush
Timer timerx; // principal para los calculos
Timer Tvista; // para retardar el paro tipo brasil
Timer Tmargen;
Timer Tled;
Timer Tesc;
long esc = 0;
long cont_margen = 0;
long t_margen = 0;

float Kp ;
int Kd ;
float reversaPD = 1;
long TiempoMuestreo = 1;
unsigned long pasado=0;      
unsigned long ahora;
double Y;                    
double errorpd;               
double errorPass=0;         
double errorAnt=0;           
double U;      
int errorx;
int tb_permitir = 0;
long contg;
long conts;
int memo;
long contv;
long tt;
char tu ;
int vc= 0;
int z;
int z1;
int z2 = 0;
       float z3;
    uint16_t ebrush; 
    uint16_t estado1;
    uint16_t estado2;
    uint16_t estado3;
    uint16_t estado4;
    uint16_t estado5;
    uint16_t estado6;
    uint16_t estado7;
    uint16_t estado8;
    uint16_t estado9;
    uint16_t estado10;
    uint16_t estado11;
    uint16_t estado12;
    uint16_t estado13;
    uint16_t estado14;
    uint16_t estado15;
    uint16_t estado16;
    
     
int16_t maxs1 = 0;
uint16_t maxs2 = 0;
uint16_t maxs3 = 0;
uint16_t maxs4 = 0;
uint16_t maxs5 = 0;
uint16_t maxs6 = 0;
uint16_t maxs7 = 0;
uint16_t maxs8 = 0;
uint16_t maxs9 = 0;
uint16_t maxs10 = 0;
uint16_t maxs11 = 0;
uint16_t maxs12 = 0;
uint16_t maxs13 = 0;
uint16_t maxs14 = 0;
uint16_t maxs15 = 0;
uint16_t maxs16 = 0;

int16_t mins1 = 0;
uint16_t mins2 = 0;
uint16_t mins3 = 0;
uint16_t mins4 = 0;
uint16_t mins5 = 0;
uint16_t mins6 = 0;
uint16_t mins7 = 0;
uint16_t mins8 = 0;
uint16_t mins9 = 0;
uint16_t mins10 = 0;
uint16_t mins11 = 0;
uint16_t mins12 = 0;
uint16_t mins13 = 0;
uint16_t mins14 = 0;
uint16_t mins15 = 0;
uint16_t mins16 = 0;

    int posicion ;
    int in_min = 0;
    long in_max = 65536;
    long out_min = 0;
    long out_max = 255;
    long v;
    long k;
    long k1;
    long k2 ;
    float time_edfon;
    float time_edfclb;
    int counter_edfon = 0;
    int counter_edfclb = 0;
    int calibrate_esc=0;
    uint16_t eboton;
    uint16_t epot;
    int esc_fin  = 0;
    int m_esc= 0;
    int memoria_inter = 0;
    int memoria_tb=0;
    int contn =0;
long j=0;
 int pp = 0;
int cont_tbpermitir=0;
int cont_mtpermitir = 0;
float k3;
int memox = 0;
int data_start = 0;
int cont_ndatos = 0;
int edf_tx = 0;

int valor_sens = 100;
int vista = 0;
long before_vista = 0;
int x0 = 0;
uint16_t e_encoder = 0;
long cont_encoder = 0;
long cont_encoder2 = 0;
long cont_marcas = 0;
int cc = 0;
int cc2 = 0;
int memoria_encoder = 0;
int yuk  = 0;
float time_muestreo = 0;
int yh = 0;
int kt = 0;
int estado1x = 0;
int estado2x = 0;
int estado3x = 0;
int estado4x = 0;
int estado5x = 0;
int estado6x = 0;
int estado7x = 0;
int estado8x = 0;
int estado9x = 0;
int estado10x = 0;
int estado11x = 0;
int estado12x = 0;
int estado13x = 0;
int estado14x = 0;
int estado15x = 0;
int estado16x = 0;
  long  TL = 0; // Guardar tiempo para led
  int calibrar_inter = 0;
  long count_deg = 0;
  int memoria_deg = 0;
  int class_memory1 = 0; 
  int class_memory2 = 0; 
  int class_memory3 = 0; 
  int xc = 1; 
  float lineal = 0; 
  
  int m = 0; 



    
    


void lectura()
{
     
  
    uno = 0; dos = 0; tres = 0; cuatro = 0;  estado1 = sensores.read_u16()/257;
   uno = 0; dos = 0; tres = 0; cuatro = 1;  estado5 = sensores.read_u16()/257;
   uno = 0; dos = 0; tres = 1; cuatro = 0;  estado9 = sensores.read_u16()/257;
   uno = 0; dos = 0; tres = 1; cuatro = 1;  estado13 = sensores.read_u16()/257;
   uno = 0; dos = 1; tres = 0; cuatro = 0;  estado3 = sensores.read_u16()/257;
   uno = 0; dos = 1; tres = 0; cuatro = 1;  estado7 = sensores.read_u16()/257;
   uno = 0; dos = 1; tres = 1; cuatro = 0;  estado11 = sensores.read_u16()/257;
   uno = 0; dos = 1; tres = 1; cuatro = 1;  estado15 = sensores.read_u16()/257;
   uno = 1; dos = 0; tres = 0; cuatro = 0;  estado2 = sensores.read_u16()/257;
   uno = 1; dos = 0; tres = 0; cuatro = 1;  estado6 = sensores.read_u16()/257;
   uno = 1; dos = 0; tres = 1; cuatro = 0;  estado10 = sensores.read_u16()/257;
   uno = 1; dos = 0; tres = 1; cuatro = 1;  estado14 = sensores.read_u16()/257;
   uno = 1; dos = 1; tres = 0; cuatro = 0;  estado4 = sensores.read_u16()/257;
   uno = 1; dos = 1; tres = 0; cuatro = 1;  estado8 = sensores.read_u16()/257;
   uno = 1; dos = 1; tres = 1; cuatro = 0;  estado12 = sensores.read_u16()/257;
   uno = 1; dos = 1; tres = 1; cuatro = 1;  estado16 = sensores.read_u16()/257;
   
   
   
     // 2 para linea negre, el robot normalmente da maximo en blanco minimo en negro
if ( line_data == 2 ) {
   
   estado1 = abs (255-estado1);
   estado2 = abs (255-estado2);
   estado3 = abs (255-estado3);
   estado4 = abs (255-estado4);
   estado5 = abs (255-estado5);
   estado6 = abs (255-estado6);
   estado7 = abs (255-estado7);
   estado8 = abs (255-estado8);
   estado9 = abs (255-estado9);
   estado10 = abs (255-estado10);
   estado11 = abs (255-estado11);
   estado12 = abs (255-estado12);
   estado13 = abs (255-estado13);
   estado14 = abs (255-estado14);
   estado15 = abs (255-estado15);
   estado16 = abs (255-estado16);  }
    
   }
   

    
    
    
    
    void timer_interrupt_pid()
{

         time_muestreo = 0.001;
         
         
      double errorD =(errorx-errorAnt);  
      float P=Kp*errorx;                       
      float D=Kd*errorD*.1;
      U=P+D;
                    
      errorAnt=errorx;         
    interrupt_pid.attach(&timer_interrupt_pid, time_muestreo); 
  
   
}






void timer_interrupt_edf()
{

    counter_edfon=counter_edfon+1;
    if ( counter_edfon == 1 ) { time_edfon = k3; }
    if ( counter_edfon == 2 ) { time_edfon = 0.019;  }
    if ( cont_tbpermitir == 0 ) { time_edfon= 0.001; }
    if ( allow == 0 ) { time_edfon= 0.001; }
    brush = !brush;        
           
    interrupt_edf.attach(&timer_interrupt_edf, time_edfon); 
    if ( counter_edfon == 2 ) { counter_edfon = 0; }
   
}
 
 

 
 


int main() {
   pc.baud(9600); 
   pc.format(8,Serial::None,1); 
   mif.period(0.002);    
   mia.period(0.002);
   mdf.period(0.002);
   mda.period(0.002);
   
   //boton.mode(PullDown);

   // Datas*****************************************************************************************************************************************************


 
   
     brushs1:  //******************************************************************************
    eboton = boton.read_u16();
    if ( eboton > 30000 )  { allow = 0; led = 0;  }
    if ( eboton < 1000 )  {  led = 1;  }

   timerb.start();
   j = timerb.read_ms();
   if (j > 3000) { esc_fin= 1; timerb.stop();   goto data_before; }
   brush = 1;
   wait_us(1200);
   brush = 0;
   wait_us(18000);
   goto brushs1;  
  
     data_before: //****************************************************************
     led3 = 0;
     eboton = boton.read_u16();
     if ( eboton > 30000 ) { wait ( .5);  goto calibrate; }
     goto data_before;
     
          
            
   
       
       
       
       

off:
mif.write(0); mdf.write(0); mia.write(0); mda.write(0); 
cont_tbpermitir = 0;
eboton = boton.read_u16();
if ( eboton > 30000  && modulo_on >= 1 ) { goto inicio_bef; }
goto off;
  
     
    
       
       
       
       
  calibrate: //***********************************************************************************
          
          Tesc.start();
   esc = Tesc.read_us();
     if ( esc <= 1200 && allow == 1 ) { brush = 1; }  // 1170 blanco - 1370 azul
   if ( esc > 1200 )  { brush = 0; }
   if ( esc >= 18000 )  {  Tesc.reset(); } 
          
   Tled.start();
   TL = Tled.read_ms();
   if ( TL >= 200 ) { Tled.reset(); }
   if ( TL <= 100 ) { led3 = 1; }
   if ( TL > 100 ) { led3 = 0; }
  
              
   lectura();
    
    eboton = boton.read_u16();
    if ( eboton > 30000 && calibrar_inter == 0) { Tled.stop(); led3 = 0; wait(.3); goto start1; }
   
                   
    //if (estado1 > maxs1) { maxs1 = estado1; }
    if (estado1 > mins1) { mins1 = estado1; }

     //if (estado2 > maxs2) { maxs2 = estado2; }
    if (estado2 >  mins2) { mins2 = estado2; }

    // if (estado3 > maxs3) { maxs3 = estado3; }
    if (estado3 > mins3) { mins3 = estado3; }

     //if (estado4 > maxs4) { maxs4 = estado4; }
    if (estado4 > mins4) { mins4 = estado4; }

    // if (estado5 > maxs5) { maxs5 = estado5; }
    if (estado5 > mins5) { mins5 = estado5; }

    // if (estado6 > maxs6) { maxs6 = estado6; }
    if (estado6 > mins6) { mins6 = estado6; }

    // if (estado7 > maxs7) { maxs7 = estado7; }
    if (estado7 > mins7) { mins7 = estado7; }

     //if (estado8 > maxs8) { maxs8 = estado8; }
    if (estado8 > mins8) { mins8 = estado8; }



    //if (estado9 > maxs9) { maxs9 = estado9; }
    if (estado9 > mins9) { mins9 = estado9; }

   // if (estado10 > maxs10) { maxs10 = estado10; }
    if (estado10 >  mins10) { mins10 = estado10; }

     //if (estado11 > maxs11) { maxs11 = estado11; }
    if (estado11 > mins11) { mins11 = estado11; }

     //if (estado12 > maxs12) { maxs12 = estado12; }
    if (estado12 > mins12) { mins12 = estado12; }

    // if (estado13 > maxs13) { maxs13 = estado13; }
    if (estado13 > mins13) { mins13 = estado13; }

     //if (estado14 > maxs14) { maxs14 = estado14; }
    if (estado14 > mins14) { mins14 = estado14; }

     //if (estado15 > maxs15) { maxs15 = estado15; }
    if (estado15 > mins15) { mins15 = estado15; }

     //if (estado16 > maxs16) { maxs16 = estado16; }
    if (estado16 > mins16) { mins16 = estado16; }
          
      maxs1 = 255+mins1;
      maxs2 = 255+mins2;
      maxs3 = 255+mins3;
      maxs4 = 255+mins4;
      maxs5 = 255+mins5;
      maxs6 = 255+mins6;
      maxs7 = 255+mins7;
      maxs8 = 255+mins8;
      maxs9 = 255+mins9;
      maxs10 = 255+mins10;
      maxs11 = 255+mins11;
      maxs12 = 255+mins12;
      maxs13 = 255+mins13;
      maxs14 = 255+mins14;
      maxs15 = 255+mins15;
      maxs16 = 255+mins16;
      
     goto calibrate;     
       
       

  //******************************************************************************************************************************************
 
  start1:
  
         Tesc.start();
   esc = Tesc.read_us();
      if ( esc <= 1200 && allow == 1 ) { brush = 1; }
   if ( esc > 1200 )  { brush = 0; }
   if ( esc >= 18000 )  {  Tesc.reset(); } 
  
  eboton = boton.read_u16();
  if ( eboton > 30000 && class_memory2 == 0 ) { class_memory3 = 0; class_memory1 = class_memory1+1; class_memory2 = 1; wait(.1);  }
  if ( eboton < 1000  && class_memory2 == 1 ) { wait(.050); class_memory2 = 0;  }
  if ( class_memory1 >= 1 ) { wait(.001); class_memory3 = class_memory3+1; }
  if ( class_memory3 >= 800) { goto showx; } 
  
goto start1;

showx:
if ( class_memory1 == 1 ) { lineal = 1; m = 1; goto conturbina; } 

while ( xc < class_memory1 ) {
    led = 1; 
    wait(.2);
    led = 0; 
    wait(.3); 
    xc = xc+1; }
    
    if ( class_memory1 == 2 ) { lineal = .1;   goto sinturbina; } 
    if ( class_memory1 == 3 ) { lineal = 0.05; goto sinturbina; } 
    if ( class_memory1 == 4 ) { lineal = 0.04; goto sinturbina; } 
    if ( class_memory1 == 5 ) { lineal = 0.03; goto sinturbina; } 
    if ( class_memory1 == 6 ) { lineal = 0.02; goto sinturbina; } 
     if ( class_memory1 == 7 ) { lineal = 0.01; goto sinturbina; } 

goto showx; 
  
  conturbina: 

  
  led= 1;
  
          Tesc.start();
   esc = Tesc.read_us();
     if ( esc <= 1350 && allow == 1 ) { brush = 1; }  // 1170 blanco - 1370 azul
   if ( esc > 1350 )  { brush = 0; }
   if ( esc >= 18000 )  {  Tesc.reset(); } 
  

  eboton = boton.read_u16();
    if ( eboton > 30000 && modulo_on >= 1) { led = 0; Tesc.stop(); brush = 0;   goto inicio_bef; }
    if ( eboton > 30000 && x0 == 0) { wait (.2); x0 = 1; }
    if ( eboton < 500 && x0 == 1 ) {led = 0; Tesc.stop(); brush = 0; wait (.06);  goto inicio_bef; }
    goto conturbina;
    
      
      sinturbina: 
  Tesc.start();
   esc = Tesc.read_us();
     if ( esc <= 2450 ) { brush = 1; }  // 1170 blanco - 1370 azul
   if ( esc > 2450 )  { brush = 0; }
   if ( esc >= 17000 )  {  Tesc.reset(); }
      
  led= 1;
  eboton = boton.read_u16();
    if ( eboton > 30000 && modulo_on >= 1) { led = 0; Tesc.stop(); brush = 0;  goto inicio_bef; }
    if ( eboton > 30000 && x0 == 0) { wait (.2); x0 = 1; }
    if ( eboton < 500 && x0 == 1 ) {led = 0; Tesc.stop(); brush = 0;  goto inicio_bef; }
    goto sinturbina;
  
        inicio_bef: //************************************************************************************************************************
      
       //  wait ( 3);  // 3 SEGUNDOS ANTES DE TURBINA 
        
        interrupt_edf.attach(&timer_interrupt_edf, time_edfon);  
        interrupt_pid.attach(&timer_interrupt_pid, time_muestreo); 
      
        
       
    
        k3 = turbina*0.00002; // tiempo turbina
    
        
            
        cont_tbpermitir = 1; // permitir uso de edf
   
        AnalogIn pot ( PA_0); // declaro mi pot despues del Serial
    
        // wait ( 2);  // 2 SEGUNDOS DESPUES DE ENCENDER TURBINA****
   
    while(1) {
    
    
   
     eboton = boton.read_u16();
     if ( eboton < 500  && modulo_on == 2 ) { goto off; }
     
    epot = pot.read_u16();
    epot =(epot - 0) * (255 - 0) / (65536 - 0) + 0; //mapear
   //if ( vista == 0) { velocidad =epot; }  
     //velocidad =epot
     
     if ( velocidad < epot ) { velocidad = velocidad+lineal;  } // Aceleracion Lineal .5 solo con turbina  
     
     
   lectura();
 
 estado1x =(estado1 - mins1 ) *  maxs1 /(255-mins1) ; 
    if ( estado1x <= 0 ) { estado1x = 0; }
    if ( estado1x >= 255 ) { estado1x = 255; }
    
    
     estado2x =(estado2 - mins2 )*  maxs2 /(255-mins2) ; 
    if ( estado2x <= 0 ) { estado2x = 0; }
    if ( estado2x >= 255 ) { estado2x = 255; }
    
     estado3x =(estado3 - mins3 )* maxs3/(255-mins3); 
    if ( estado3x <= 0 ) { estado3x = 0; }
    if ( estado3x >= 255 ) { estado3x = 255; }
    
     estado4x =(estado4 - mins4 )* maxs4/(255-mins4); 
    if ( estado4x <= 0 ) { estado4x = 0; }
    if ( estado4x >= 255 ) { estado4x = 255; }
    
     estado5x =(estado5 - mins5 )* maxs5/(255-mins5); 
    if ( estado5x <= 0 ) { estado5x = 0; }
    if ( estado5x >= 255 ) { estado5x = 255; }
    
     estado6x =(estado6 - mins6 )* maxs6/(255-mins6); 
    if ( estado6x <= 2 ) { estado6x = 0; }
    if ( estado6x >= 255 ) { estado6x = 255; }
    
     estado7x =(estado7 - mins7 )* maxs7/(255-mins7); 
    if ( estado7x <= 0 ) { estado7x = 0; }
    if ( estado7x >= 255 ) { estado7x = 255; }
    
     estado8x =(estado8 - mins8 )* maxs8/(255-mins8); 
    if ( estado8x <= 0 ) { estado8x = 0; }
    if ( estado8x >= 255 ) { estado8x = 255; }
    
     estado9x =(estado9 - mins9 )* maxs9/(255-mins9); 
    if ( estado9x <= 0 ) { estado9x = 0; }
    if ( estado9x >= 255 ) { estado9x = 255; }
    
     estado10x =(estado10 - mins10 )* maxs10/(255-mins10); 
    if ( estado10x <= 0 ) { estado10x = 0; }
    if ( estado10x >= 255 ) { estado10x = 255; }
    
     estado11x =(estado11 - mins11 )* maxs11/(255-mins11); 
    if ( estado11x <= 0 ) { estado11x = 0; }
    if ( estado11x >= 255 ) { estado11x = 255; }
    
     estado12x =(estado12 - mins12 )* maxs12/(255-mins12); 
    if ( estado12x <= 0 ) { estado12x = 0; }
    if ( estado12x >= 255 ) { estado12x = 255; }
    
     estado13x =(estado13 - mins13 )* maxs13/(255-mins13); 
    if ( estado13x <= 0 ) { estado13x = 0; }
    if ( estado13x >= 255 ) { estado13x = 255; }
    
     estado14x =(estado14 - mins14 )* maxs14/(255-mins14); 
    if ( estado14x <= 0 ) { estado14x = 0; }
    if ( estado14x >= 255 ) { estado14x = 255; }
    
     estado15x =(estado15 - mins15 )* maxs15/(255-mins15); 
    if ( estado15x <= 0 ) { estado15x = 0; }
    if ( estado15x >= 255 ) { estado15x = 255; }
    
      estado16x =(estado16 - mins16 )* maxs16/(255-mins16); 
    if ( estado16x <= 0 ) { estado16x = 0; }
    if ( estado16x >= 255 ) { estado16x = 255; }

    

  //  pc.printf (" e1: %i.\r\n", estado8); // imprime potenciometro

 
if ( estado5x > valor_sens  ) { Kp = Kpn; Kd = Kdn; }
if ( estado12x > valor_sens ) { Kp = Kpn; Kd = Kdn; }

if ( estado8x > valor_sens ) { Kp = Kpn; Kd = Kde; }
if ( estado9x > valor_sens ) { Kp = Kpn; Kd = Kde; }

if ( estado8x > valor_sens && estado9x > valor_sens ) { memo = 1; }

if ( estado8x > valor_sens && estado9x < valor_sens ) { memo = 2; }
if ( estado9x > valor_sens && estado8x < valor_sens ) { memo = 3; }


    
        // si hay intersecciones o no tiene que leer marcas a los lados 
        
       // Normal  if ( memo == 1 ) { Kp = Kpn; Kd = Kde;  errorx = estado6x*(-.4)+estado7x*(-.2)+estado8x*(-.1) + estado9x*(.1) +estado10x*(.2)+estado11x*(.4);}
       // Normal  if ( memo == 2 ) { Kp = Kpn; Kd = Kdn;   errorx = estado1x*(-1) + estado2x*(-2)+ estado3x*(-1)+estado4x*(-.8)+ estado5x*(-.6)+ estado6x*(-.4)+estado7x*(-.2)+estado8x*(-.1) + estado9x*(.1) +estado10x*(.2)+estado11x*(.4)+estado12x*(.6)+ estado13x*(.8)+ estado14x*(1)+estado15x*(2)+estado16x*(1);}
        
          if ( memo == 1 ) {   errorx = estado7x*(-.2)+estado8x*(-.1) + estado9x*(.1) +estado10x*(.2);}
         
         if ( memo == 2 ) {  
          errorx = estado1x*(-1) + estado2x*(-2)+ estado3x*(-1)+estado4x*(-.8)+ estado5x*(-.6)+ estado6x*(-.4)+estado7x*(-.2)+estado8x*(-.1) ;} 
         
         if ( memo == 3 ) {   
          errorx = estado9x*(.1) +estado10x*(.2)+estado11x*(.4)+estado12x*(.6)+ estado13x*(.8)+ estado14x*(1)+estado15x*(2)+estado16x*(1);}
         
        /*
             errorx = estado1x*(-1) + estado2x*(-2)+ estado3x*(-1)+estado4x*(-.8)+ estado5x*(-.6)+ estado6x*(-.4)+estado7x*(-.2)+estado8x*(-.1) + estado9x*(.1) +estado10x*(.2)+estado11x*(.4)+estado12x*(.6)+ estado13x*(.8)+ estado14x*(1)+estado15x*(2)+estado16x*(1);
             if ( memo == 1 ) { Kp = Kpn; Kd = Kde;  }
             if ( memo == 2 ) { Kp = Kpn; Kd = Kdn;   }
          */   
          
          
if ( estado2x > valor_sens   ) { posicion = 0; contg = 0;}
if ( estado15x > valor_sens ) { posicion = 0; contg = 0;}

//if ( estado1x > valor_sens ) {  posicion = 1;}  // que vea extremo, 8 y 9 no vean
//if ( estado16x > valor_sens  ) {  posicion = 16;}

if ( estado1x > valor_sens && estado8x < valor_sens && estado9x < valor_sens ) {   posicion = 1;}  // que vea extremo, 8 y 9 no vean
if ( estado16x > valor_sens && estado8x < valor_sens && estado9x < valor_sens ) {   posicion = 16;}


// parte para poder leer angulos
if ( estado16x > valor_sens)  { count_deg = 0; memoria_deg = 16;}
if ( memoria_deg == 16 && estado16x < valor_sens && estado7x < valor_sens && estado8x < valor_sens && estado9x < valor_sens && estado10x < valor_sens )  {  memoria_deg = 0; posicion  = 16;}

if ( estado1x  > valor_sens  )  {  count_deg = 0; memoria_deg = 1;}
if ( memoria_deg == 1 && estado1x < valor_sens && estado7x < valor_sens && estado8x < valor_sens && estado9x < valor_sens && estado10x < valor_sens )  {   memoria_deg = 0; posicion  = 1;}

if ( estado1 < valor_sens && memoria_deg > 0 ) { count_deg = count_deg+1; led = 1;}
if ( estado16 < valor_sens && memoria_deg > 0 ) { count_deg = count_deg+1; led = 1;}
if ( count_deg >= 1000) { memoria_deg = 0; led = 0; } 
//// parte para poder leer angulos
  
     
 if ( U >= 255 )  { U =  255; }
 if ( U <= -255 ) { U = -255; }
       
int pwm1 = velocidad - (U);
int pwm2 = velocidad + (U);

if ( pwm1 >= 255 ) { pwm1 = 255; }
if ( pwm2 >= 255 ) { pwm2 = 255; }

float pwm11 = pwm1*.004;
float pwm22 = pwm2*.004;

if ( pwm11 >= 1 ) { pwm11 = 1; }
if ( pwm22 >= 1 ) { pwm22 = 1; }



float r1 = abs(pwm11);
float r2 = abs (pwm22);

if ( r1 >= 1 ) { r1 = 1; }
if ( r2 >= 1 ) { r2 = 1; }

float reversa1 = reversa*.004;
float frente1 = frente*.004;
if ( reversa1 >= 1 ) { reversa1 = 1; }
if ( frente1 >= 1 ) { frente1 = 1; }


if ( posicion == 16 ) {
 
contg = contg+1;
if ( contg < xu) { 
 mda.write(reversa1); mdf.write(0);
 mif.write (frente1); mia.write (0); }
 
 if ( contg > xu) { 
 mda.write(0); mdf.write(0); 
 mif.write(frente1); mia.write(0); } }
           
  
  
if ( posicion == 1 )  {
    
contg = contg+1;
if ( contg < xu) {
  mdf.write(frente1); mda.write(0); 
  mia.write(reversa1); mif.write(0); }
  
  if ( contg > xu) {
  mdf.write(frente1); mda.write(0); 
  mif.write(0); mia.write(0);  } }
  
  
if ( posicion != 16 && posicion != 1 ) {  


if ( pwm11 < 0 ) {  mda.write(r1); mdf.write(0); }

if ( pwm22 < 0 ) {  mia.write(r2); mif.write(0); }

if ( pwm11 == 0 ) { mda.write(0); mdf.write(0);   }

if ( pwm22 == 0 ) { mia.write(0); mif.write(0);   } 

if ( pwm11 > 0 )  { mdf.write(pwm11); mda.write(0); }
           
if ( pwm22 > 0 )  { mif.write(pwm22); mia.write(0); } 

  }}}