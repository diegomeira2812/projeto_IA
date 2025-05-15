#include <stdio.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 512
#define QtddSensoresProx 8
#define QtddLeds 10
#define TamanhoTexto 256

#define NUM_BOXES 3         // Número de caixas (ajuste conforme necessário)
#define PUSH_THRESHOLD 0.25 // Distância (em metros) para considerar que a caixa foi alcançada

// Parâmetros para odometria (ajuste de acordo com o seu modelo)
#define WHEEL_RADIUS 0.02   // Raio da roda, em metros
#define AXLE_LENGTH 0.053   // Distância entre as rodas, em metros

int main(int argc, char **argv) {
  int i;
  char texto[TamanhoTexto] = {0};
  double LeituraSensorProx[QtddSensoresProx];
  
  wb_robot_init();
  
  // ================= Obtenção das caixas via supervisor ==================
  // Certifique-se de definir as caixas no mundo com DEF "CAIXA00", "CAIXA01", "CAIXA02", etc.
  WbNodeRef caixas[NUM_BOXES];
  caixas[0] = wb_supervisor_node_get_from_def("CAIXA00");
  caixas[1] = wb_supervisor_node_get_from_def("CAIXA01");
  caixas[2] = wb_supervisor_node_get_from_def("CAIXA02");
  
  // ================= Configurar os motores do e-puck ==================
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito  = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);
  
  // ================= Configurar sensores de proximidade ==================
  // (Para este exemplo, os sensores são usados para debug ou poderão vir a ser empregados para evitar colisões)
  WbDeviceTag SensorProx[QtddSensoresProx];
  char nomeSensor[10] = {0};
  for (i = 0; i < QtddSensoresProx; i++){
    sprintf(nomeSensor, "ps%d", i);
    SensorProx[i] = wb_robot_get_device(nomeSensor);
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }
  
  // ================= Configurar o LED (para debug visual) ==================
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0], -1);
  
  // ================= Variáveis para odometria do robô ==================
  double robot_x = 0.0, robot_z = 0.0;     // Posição estimada (x e z) do robô
  double robot_theta = 0.0;                // Orientação (heading); assume que o robô inicia apontando para z+

  // ================= Vetor para marcar as caixas já empurradas ==================
  int checked[NUM_BOXES] = {0};  
  
  // ================= Loop principal ==================
  while (wb_robot_step(TIME_STEP) != -1) {
    
    // Limpa o buffer de debug a cada iteração
    texto[0] = '\0';
    
    // ---- (Opcional) Leitura dos sensores de proximidade para debug ----
    for (i = 0; i < QtddSensoresProx; i++){
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]);
      size_t len = strlen(texto);
      snprintf(texto + len, TamanhoTexto - len, "|%d: %6.0f ", i, LeituraSensorProx[i] - 60);
    }
    
    // ---- Selecionar a caixa alvo, dentre as não verificadas, mais próxima (baseada na posição global da caixa e na posição estimada do robô) ----
    int targetBox = -1;
    double minDistance = 1e9;
    for(i = 0; i < NUM_BOXES; i++){
      if (!checked[i]) {
        const double *boxPos = wb_supervisor_node_get_position(caixas[i]);
        double dx = boxPos[0] - robot_x;
        double dz = boxPos[2] - robot_z;
        double dist = sqrt(dx*dx + dz*dz);
        if (dist < minDistance) {
          minDistance = dist;
          targetBox = i;
        }
      }
    }
    
    // ---- Debug: Mostrar posição estimada do robô ----
    size_t len = strlen(texto);
    snprintf(texto + len, TamanhoTexto - len, "|Robô (odom): x=%.2f, z=%.2f, θ=%.2f ", robot_x, robot_z, robot_theta);
    
    // ---- Se todas as caixas já foram empurradas, o robô para ----
    if(targetBox == -1) {
      wb_motor_set_velocity(MotorEsquerdo, 0);
      wb_motor_set_velocity(MotorDireito, 0);
      printf("Todas as caixas foram empurradas.\n");
    } else {
      // Obter posição da caixa alvo
      const double *targetPos = wb_supervisor_node_get_position(caixas[targetBox]);
      len = strlen(texto);
      snprintf(texto + len, TamanhoTexto - len, "|Target CAIXA%d: x=%.2f, z=%.2f ",
               targetBox, targetPos[0], targetPos[2]);
      
      // Exibição do debug
      printf("%s\n", texto);
      fflush(stdout);
      
      // Calcular distância e erro horizontal (diferença em x) entre a caixa e a posição estimada do robô
      double dx = targetPos[0] - robot_x;
      double dz = targetPos[2] - robot_z;
      double distance = sqrt(dx * dx + dz * dz);
      //double error = dx;  // Estratégia simples: usar o erro em x para ajustar o rumo

      static double ultima_pos[NUM_BOXES][3] = {{0}};

      if(distance < PUSH_THRESHOLD) {
        const double *pos_atual = wb_supervisor_node_get_position(caixas[targetBox]);
        
        if (ultima_pos[targetBox][0] == 0 && ultima_pos[targetBox][2] == 0) {
          // Primeira aproximação: salva a posição e começa empurrar
          ultima_pos[targetBox][0] = pos_atual[0];
          ultima_pos[targetBox][1] = pos_atual[1];
          ultima_pos[targetBox][2] = pos_atual[2];
      
          wb_motor_set_velocity(MotorEsquerdo, 6.28);
          wb_motor_set_velocity(MotorDireito, 6.28);
          printf("Empurrando a CAIXA%d (distância=%.2f m)!\n", targetBox, distance);
          
          // NÃO marque como verificada ainda!
          // checked[targetBox] = 1;
        } else {
          double dx = pos_atual[0] - ultima_pos[targetBox][0];
          double dz = pos_atual[2] - ultima_pos[targetBox][2];
          double deslocamento = sqrt(dx * dx + dz * dz);
      
          if (deslocamento > 0.01) {
            printf("Caixa leve encontrada! CAIXA%d se moveu.\n", targetBox);
          } else {
            printf("CAIXA%d não se moveu. Ignorando.\n", targetBox);
          }
      
          checked[targetBox] = 1;
          wb_motor_set_velocity(MotorEsquerdo, 0);
          wb_motor_set_velocity(MotorDireito, 0);
        }
      }
    }
    
    // ---- Atualiza a odometria com base nos comandos atuais ----
    // Supondo que os comandos enviados sejam os valores atuais dos motores (em rad/s).
    // A velocidade linear de cada roda é: v = ω * r.
    // O robô possui velocidade linear média e rotação baseada na diferença entre as rodas.
    double dt = TIME_STEP / 1000.0;  // tempo em segundos
    // Os valores atuais "left_speed" e "right_speed" já foram definidos em cada ramo.
    // Para atualização, vamos supor que usamos os últimos comandos enviados:
    // (Veja que, para casos de empurrão ou controle, os mesmos comandos foram enviados)
    // Aqui, recupere os comandos enviados; 
    // Para simplificar, vamos considerar os comandos que acabamos de definir:
    double v_left = 0, v_right = 0;
    // Se estivermos empurrando, os comandos são 6.28;
    // Caso contrário, são os valores calculados no controle.
    // Vamos usar um "if" para atualizar a odometria de forma aproximada.
    if (targetBox == -1) {
      // Robô parado
      v_left = 0;
      v_right = 0;
    } else {
      const double *targetPos = wb_supervisor_node_get_position(caixas[targetBox]);

      // Se estiver empurrando:
      if (minDistance < PUSH_THRESHOLD) {
        v_left = 6.28;
        v_right = 6.28;
      } else {
        // Se não estiver empurrando, use os comandos do controle proporcional.
        // Como estes foram enviados logo acima na cláusula "else", vamos assumir
        // que as variáveis left_speed e right_speed contêm os comandos atuais.
        // Por simplicidade, vamos declarar essas variáveis locais aqui:
        double error = targetPos[0] - robot_x;
        double Kp = 2.0;
        double base_speed = 3.0;
        double correction = Kp * error;
        v_left = base_speed - correction;
        v_right = base_speed + correction;
      }
    }
    
    // Atualização da odometria:
    double v_left_linear = v_left * WHEEL_RADIUS;
    double v_right_linear = v_right * WHEEL_RADIUS;
    double v_robot = (v_left_linear + v_right_linear) / 2.0;
    double omega_robot = (v_right_linear - v_left_linear) / AXLE_LENGTH;
    
    robot_theta += omega_robot * dt;
    // Se considerarmos que theta=0 é apontando para o eixo z positivo:
    robot_x += v_robot * sin(robot_theta) * dt;
    robot_z += v_robot * cos(robot_theta) * dt;
    
    // Piscar o LED para indicar atividade
    wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);
  }
  
  wb_robot_cleanup();
  return 0;
}
