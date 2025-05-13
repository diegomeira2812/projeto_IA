#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 512
#define QtddSensoresProx 8
#define QtddLeds 10
#define TamanhoTexto 256


int main(int argc, char **argv) {
  int i = 0;
  char texto[TamanhoTexto] = {0};
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
  
  wb_robot_init();
  
  // Referência ao objeto "CAIXA" no mundo (lembre-se: no mundo, defina o objeto com DEF CAIXA)
  WbNodeRef caixa = wb_supervisor_node_get_from_def("CAIXA");
  
  // Configuração dos motores
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);
  
  // Configuração dos sensores de proximidade
  WbDeviceTag SensorProx[QtddSensoresProx];
  char nomeSensor[10] = {0};
  
  for (i = 0; i < QtddSensoresProx; i++) {
    sprintf(nomeSensor, "ps%d", i);
    SensorProx[i] = wb_robot_get_device(nomeSensor);
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }
  
  // Configuração do LED
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0], -1);
  
  while (wb_robot_step(TIME_STEP) != -1) {
    
    // Leitura dos sensores de proximidade
    for (i = 0; i < QtddSensoresProx; i++) {
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]) - 60;
      sprintf(texto, "%s|%d: %6.0f  ", texto, i, LeituraSensorProx[i]);
    }
    
    // Captura a posição da caixa via supervisor
    const double *PosicaoCaixa = wb_supervisor_node_get_position(caixa);
    sprintf(texto, "%s|caixa em x=%5.2f, y=%5.2f, z=%5.2f", texto,
            PosicaoCaixa[0], PosicaoCaixa[1], PosicaoCaixa[2]);
    
    // Mostra os valores lidos para debug
    printf("%s\n", texto);
    fflush(stdout);
    
    // Pisca o LED
    wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);
    
    // Verifica se algum sensor de proximidade atingiu o limiar de parede
    double maxValor = 0.0;
    for (i = 0; i < QtddSensoresProx; i++) {
      if (LeituraSensorProx[i] > maxValor)
        maxValor = LeituraSensorProx[i];
    }
    
    if (maxValor >= 1000) {
      // Pare o robô
      printf("Parede detectada! Parando o robô...\n");
      fflush(stdout);
      wb_motor_set_velocity(MotorEsquerdo, 0);
      wb_motor_set_velocity(MotorDireito, 0);
    }
    else {
      // Comportamento normal: define o sentido de rotação com base na posição da caixa
      if (PosicaoCaixa[0] * PosicaoCaixa[1] < 0)
        AceleradorDireito = -1.0, AceleradorEsquerdo = 1.0;
      else
        AceleradorDireito = 1.0, AceleradorEsquerdo = -1.0;
      
      wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
      wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);
    }
    
    // Limpa a string para a próxima iteração
    memset(texto, 0, TamanhoTexto);
  }
  
  wb_robot_cleanup();
  return 0;
}