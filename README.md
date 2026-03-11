# 🚀 Planta de Levitação a Ar - Controle Digital com ESP32

Este repositório contém o firmware e a documentação para uma **Planta de Levitação a Ar**, desenvolvida como parte de um estudo sobre **Controle Digital de Sistemas Dinâmicos**. O projeto utiliza um microcontrolador ESP32 para estabilizar a altura de um objeto dentro de um tubo através do controle de fluxo de ar.

[Image of PID control system block diagram]

## 📝 Visão Geral do Projeto

O objetivo fundamental é manter um objeto (esfera) em uma altura específica (*setpoint*), utilizando uma ventoinha (fan) como atuador e um sensor laser de tempo de voo (ToF) como feedback. 

A planta serve como plataforma experimental para:
* Modelagem matemática de sistemas pneumáticos.
* Implementação de controladores digitais (PID, Espaço de Estados).
* Análise de estabilidade e resposta em frequência.

## 🛠️ Hardware Utilizado

* **Microcontrolador:** ESP32 (DevKit V1).
* **Sensor de Altura:** VL53L0X (Time-of-Flight) via I2C.
* **Atuador:** Ventoinha DC controlada via PWM (LEDC do ESP32).
* **Driver de Potência:** Circuito MOSFET com lógica de acionamento.

## 💻 Arquitetura do Firmware

O software foi desenvolvido utilizando o **ESP-IDF v5.x** sobre o sistema operacional de tempo real **FreeRTOS**, garantindo determinismo no laço de controle.

### Principais Componentes e Tasks:
1. **`app_serial_task`**: Gerencia a comunicação USB-Serial para telemetria e recepção de comandos do MATLAB.
2. **`levitador_task`**: Executa o algoritmo de controle em malha fechada em alta frequência.
3. **`height_sensor_init`**: Driver modular para interface com o sensor VL53L0X.
4. **`PWM/LEDC Control`**: Geração de sinal PWM por hardware com resolução de 10 bits.

[Image of FreeRTOS task communication architecture]

## 📊 Integração MATLAB/Simulink

O sistema está configurado para comunicação bidirecional com o MATLAB:
* **Comando `p[valor]\n`**: Define o Duty Cycle do PWM (0-100%).
* **Comando `s\n`**: Solicita a leitura atual do sensor de altura.

Isto permite a coleta de dados para identificação de sistemas e a sintonização fina dos parâmetros do controlador.

## 🚀 Como Compilar e Rodar

1. Certifique-se de ter o **ESP-IDF** instalado e configurado.
2. Clone o repositório:
   ```bash
   git clone [https://github.com/joaomarcosvilar/levitator_esp32.git](https://github.com/joaomarcosvilar/levitator_esp32.git)