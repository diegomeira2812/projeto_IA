/*
  Exemplo introdutório de uso do WeBots
 */


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>
#include <webots/position_sensor.h> //encoder
/*
 * You may want to add macros here.
 */

//TIME_STEP é o incremento de tempo usado na simulação
//512 é um valor MUITO alto.... mas ajuda para ler o que está sendo mostrado no console
//para a simulação final, melhor usar valores menores.... 16 ou 32!
#define TIME_STEP 512


#define QtddSensoresProx 8
#define QtddLeds 10
#define TamanhoTexto 256
#define QtddCaixa 20

void delay(int time_milisec)
  {
    double currentTime, initTime, Timeleft;
    double timeValue = (double)time_milisec/1000;
    initTime = wb_robot_get_time();
    Timeleft =0.00;
    while (Timeleft < timeValue)
    {
      currentTime = wb_robot_get_time();
      Timeleft=currentTime-initTime;
      wb_robot_step(TIME_STEP);
    }
  }

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

void caixaEncontrada() {
  printf("entrou");
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");
  
  //config leds
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0],-1);
  
    
  while(wb_robot_step(TIME_STEP) != -1){
    wb_motor_set_velocity(MotorEsquerdo, 6.28); // processo para girar o robo
    wb_motor_set_velocity(MotorDireito, -6.28);
    
    //pisca o led
    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1); 
  }
}

int main(int argc, char **argv) {

  double pos_x = 0, pos_y = 0, theta = 0; // posição estimada e orientação
  double prev_encoder_esq = 0, prev_encoder_dir = 0;
  const double R = 0.0205;  // raio da roda (em metros)
  const double L = 0.053;   // distância entre rodas (em metros)

  int i=0;
  char texto[TamanhoTexto]={0};
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;
  double pos_iniciais[QtddCaixa][3];
  

  /* necessary to initialize webots stuff */
  wb_robot_init();
 
   /*
    * You should declare here WbDeviceTag variables for storing
    * robot devices like this:
    *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
    *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
    */
  
  
  
  //exemplo para referenciar um objeto no mundo
  //    Alterar a proprieddade SUPERVISOR do e-Puck para TRUE
  //    Mudar o DEF da Wooden Box para CAIXA
   WbNodeRef caixa[QtddCaixa];

   char nomeCaixa[10]={0};
   
   for(i=0;i<QtddCaixa;i++){
       sprintf(nomeCaixa,"CAIXA%02d",i+1); //form os nomes dos sensores
       caixa[i] = wb_supervisor_node_get_from_def(nomeCaixa);
       if(caixa[i]!=NULL)
          printf("%2d. %s  -  %p\n",i,nomeCaixa,(void*)caixa[i]);
       else
          printf("Falha ao carregar a posição da %s\n",nomeCaixa);
    }
    printf("\n\n CAIXAS OK  \n\n");

    //lendo posiçao das caixas
    printf("           X       Y      Z\n");
    for(i=0;i<QtddCaixa;i++){
      const double *PosicaoCaixa = wb_supervisor_node_get_position(caixa[i]);
      for (int j = 0; j < 3; j++){
        pos_iniciais[i][j] = PosicaoCaixa[j];
      }
      printf("CAIXA%02d %5.2f   %5.2f  %5.2f\n\n",i+1,PosicaoCaixa[0],PosicaoCaixa[1],PosicaoCaixa[2]);
    }
  
  
  //configurando MOTORES
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");

  WbDeviceTag encoder_esq = wb_robot_get_device("left wheel sensor");
  WbDeviceTag encoder_dir = wb_robot_get_device("right wheel sensor");

  wb_position_sensor_enable(encoder_esq, TIME_STEP);
  wb_position_sensor_enable(encoder_dir, TIME_STEP);



  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);

  //motores parados
  wb_motor_set_velocity(MotorEsquerdo,0);
  wb_motor_set_velocity(MotorDireito,0);

  
   //configura Sensores de Proximidade
   WbDeviceTag SensorProx[QtddSensoresProx];
   char nomeSensor[10]={0};
   
   for(i=0;i<QtddSensoresProx;i++){
       sprintf(nomeSensor,"ps%d",i); //form os nomes dos sensores
       SensorProx[i] = wb_robot_get_device(nomeSensor);
       wb_distance_sensor_enable(SensorProx[i],TIME_STEP);
    }


    

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
    wb_robot_step(TIME_STEP);  // primeiro passo para garantir leitura válida
    prev_encoder_esq = wb_position_sensor_get_value(encoder_esq);
    prev_encoder_dir = wb_position_sensor_get_value(encoder_dir);
  
  while (wb_robot_step(TIME_STEP) != -1) {

    //lendo sensores de proximidade
    for(i=0;i<QtddSensoresProx;i++){
       LeituraSensorProx[i]= wb_distance_sensor_get_value(SensorProx[i])-60;
       printf("|ps%d: %6.0f  ",i,LeituraSensorProx[i]);
    }


    double curr_encoder_esq = wb_position_sensor_get_value(encoder_esq);
    double curr_encoder_dir = wb_position_sensor_get_value(encoder_dir);

    double delta_esq = curr_encoder_esq - prev_encoder_esq;
    double delta_dir = curr_encoder_dir - prev_encoder_dir;

    prev_encoder_esq = curr_encoder_esq;
    prev_encoder_dir = curr_encoder_dir;

    double d_left = R * delta_esq;
    double d_right = R * delta_dir;
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / L;

    theta += d_theta;
    pos_x += d_center * cos(theta);
    pos_y += d_center * sin(theta);

    printf("|Posicao: x=%.2f y=%.2f theta=%.2f rad", pos_x, pos_y, theta);

    //mostrando valores lidos
    printf("%s\n",texto);

    //colisao
    if (LeituraSensorProx[0] > 200 || LeituraSensorProx[7] > 200){
      printf("Colisao\n");
      // Verifica se alguma caixa foi movida
      for (i = 0; i < QtddCaixa; i++) {
        const double *novaPos = wb_supervisor_node_get_position(caixa[i]);
        double dx = novaPos[0] - pos_iniciais[i][0];
        double dz = novaPos[2] - pos_iniciais[i][2]; // ignorando Y (altura)
        double deslocamento = sqrt(dx*dx + dz*dz);
      
        if (deslocamento > 0.02) { // limiar de movimento (2 cm)
          wb_motor_set_velocity(MotorEsquerdo,0); // para os motores
          wb_motor_set_velocity(MotorDireito,0);
          printf("Caixa leve encontrada! É a CAIXA%02d\n", i+1);
          caixaEncontrada();
          break;
        }
      }
      wb_motor_set_velocity(MotorEsquerdo,0); // para os motores
      wb_motor_set_velocity(MotorDireito,0);

      wb_motor_set_velocity(MotorEsquerdo, 6.28); // processo para girar o robo
      wb_motor_set_velocity(MotorDireito, -6.28);
      delay(1500);

      wb_motor_set_velocity(MotorEsquerdo,0); // para os motores
      wb_motor_set_velocity(MotorDireito,0);
    }
    
    else if (LeituraSensorProx[6] > 200 || LeituraSensorProx[4] > 200){
      printf("Colisao lateral");
      for (i = 0; i < QtddCaixa; i++) {
        const double *novaPos = wb_supervisor_node_get_position(caixa[i]);
        double dx = novaPos[0] - pos_iniciais[i][0];
        double dz = novaPos[2] - pos_iniciais[i][2]; // ignorando Y (altura)
        double deslocamento = sqrt(dx*dx + dz*dz);
      
        if (deslocamento > 0.02) { // limiar de movimento (2 cm)
          wb_motor_set_velocity(MotorEsquerdo,0); // para os motores
          wb_motor_set_velocity(MotorDireito,0);
          printf("Caixa leve encontrada! É a CAIXA%02d\n", i+1);
          caixaEncontrada();
          break;
        }
       }
      wb_motor_set_velocity(MotorEsquerdo,0); // para os motores
      wb_motor_set_velocity(MotorDireito,0);
      
      wb_motor_set_velocity(MotorEsquerdo, 6.28); // processo para girar o robo
      wb_motor_set_velocity(MotorDireito, -6.28);
      delay(500);

      wb_motor_set_velocity(MotorEsquerdo,0); // para os motores
      wb_motor_set_velocity(MotorDireito,0);
    }
    
    /*else if (LeituraSensorProx[1] > 150 || LeituraSensorProx[2] > 150){
      printf("Colisao lateral");
      wb_motor_set_velocity(MotorEsquerdo,0); // para os motores
      wb_motor_set_velocity(MotorDireito,0);
      
      wb_motor_set_velocity(MotorEsquerdo, -6.28); // processo para girar o robo
      wb_motor_set_velocity(MotorDireito, 6.28);
      delay(500);

      wb_motor_set_velocity(MotorEsquerdo,0); // para os motores
      wb_motor_set_velocity(MotorDireito,0);
    }*/
    else{
      wb_motor_set_velocity(MotorEsquerdo, 6.28* AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito , 6.28* AceleradorDireito);
    }

    //define o sentido de rotação em função da posição da caixa
    //lembre-se, é apenas um exemplo!
    /*if(PosicaoCaixa[0]*PosicaoCaixa[1]<0)
      AceleradorDireito= -1.0, AceleradorEsquerdo= 1.0;
    else
      AceleradorDireito= 1.0, AceleradorEsquerdo= -1.0;*/
    
 
    
    
    //limpa texto para a proxima iteração
    memset(texto,0,TamanhoTexto);

  };


  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */

  wb_robot_cleanup();

  return 0;

}