# 22b - AV1 - Motor de passo

Nesta avaliação vocês irão criar um sistema que controla um motor de passos.

## Descricão

Periféricos:

- PIO
- RTT
- OLED

freeRTOS:

- Task
- Queue
- Semaphore

Motor de passos:

- Motor de passo **28BYJ-48**
  - Quatro fases
  - `0,17578125` graus por passo / `2048` passos para completar `360`º

### O que é um motor de passo?

O motor de passo é um motor elétrico sem bobinas (_brushless_) que permite movimentarmos o rotor (a parte que gira) em passos. Os passos são valores em graus que dependem da construção do motor e de como ele é acionado.

O `gif` a seguir ilustra um motor de passos em movimento, no caso ele possui quatro fases. Ao lado está o `gif` que mostra o driver e o motor utilizados nessa avaliação:

![image alt <](https://upload.wikimedia.org/wikipedia/commons/thumb/6/67/StepperMotor.gif/400px-StepperMotor.gif)
![image alt >](driver_motordepasso.gif)

Notem que para fazermos o motor girar é preciso acionar as fases em sequência: `1000` -> `0100` -> `0010` -> `0001` e então repetir esse ciclo, X vezes até obtermos o ângulo de deslocamento esperado. Se o movimento for o oposto: `1000` -> `0001` -> `0010` -> `0100` -> ... o motor irá girar para o lado contrário.

Cada passo (passagem entre `1000` <- -> `0100`) aplica um movimento no rotor de alguns graus (no caso do motor que iremos usar `0,17578125`).

<span style="color:red">OBS:</span> **O tempo mínimo entre `fases` para esse motor é de `5ms`, menos que isso o motor não consegue girar.**

### Interface

A interface do motor de passos com o microcontrolador vai se dar através do acionamento de quatro pinos que é conectado a um módulo de potência:

![](uc-motor.png)

- Alimente o motor com +5V e GND.
- Escolha quatro pinos do uC e conecte as fases.

### Sugestão de montagem:

<p align="center">
  <img src="labels.svg" alt="drawing" width="300"/>
</p>


### Funcionalidade

A entrega deve ser um sistema que permite controlarmos o motor de passos, fazendo-o girar em valores de ângulo pré estabelecidos. O controle do sistema será realizado via a placa OLED e seus botões (não vamos usar os LEDs). O display deve exibir qual foi a opção selecionada:

- BTN 1: Gira o motor 45° no sentido horário
- BTN 2: Gira o motor 90° no sentido horário
- BTN 3: Gira o motor 180° no sentido horário
- 
O diagrama a seguir ilustra a Interface homem-máquina (IHM) prevista:

![](oled.png)

[![](https://img.youtube.com/vi/owp5Dj5Lu-U/maxresdefault.jpg)](https://youtu.be/owp5Dj5Lu-U)

### Firmware

Vocês devem desenvolver o firmware como indicado a seguir:

![](firmware.png)

- **O código base fornecido é o `RTOS-OLED-Xplained-Pro` já com o RTT adicionado no wizard.**

Onde:

- Botões:

  - devem ser configurados para funcionar com `callback`
  - devem enviar por uma fila `xQueueModo` um inteiro que representa o ângulo de rotação e a direção
    - exemplo: 45, 90, 180, ....

- `task_modo`
  - Responsável por ler a opcão do usuário via `xQueueModo`
  - Deve fazer a conta e traduzir ângulo em passos
  - Enviar a informação para a fila `xQueueSteps` no seguinte formato `(direção)(número de passos)`:
    - exemplo para 90 -> `(90 / 0,17578125)` -> `512` passos
    - o dado que vai para fila é `512`
  - Exibe no OLED o valor selecionado em graus
  
- `task_motor`
  - Recebe pela fila a quantidade de passos e o sentido de rotação e aciona o motor
  - Deve usar o RTT para calcular o tempo que cada fase fica acionada (velocidade do motor)
    - A comunicação RTT e task deve ser feita via semáforo `xSemaphoreRTT`

- RTT
  - Irá dar a base de tempo que cada fase fica acionada
  - Configurado em modo alarme para contar 5ms
  - Toda vez que estourar o alarme, liberar semáforo `xSemaphoreRTT`

![](rtt.jpg)

### Dicas

1. Inicialize os pinos do motor, testando os movimentos, fazendo-o girar no sentiro horário e anti-horário
1. Faça o teste do OLED, `task_modo`, callback dos botões, `xQueueModo`
1. Crie a fila `xQueueSteps` e comece enviar o dado dos passos
1. Crie a `task_motor`
1. Receba os dados da fila `xQueueSteps` e acione o motor
1. Criei O rtt para operar com IRQ a cada 5ms
