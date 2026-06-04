# BTS7960 (IBT-2) Dual PWM Engine Driver for Zephyr RTOS

*Leia em outros idiomas: [English](README.md)*

Este módulo fornece um driver estático em C modular e otimizado para o controle de motores DC usando a ponte H de alta corrente **BTS7960** (comumente vendida sob a placa de interface **IBT-2**). 

O controle é feito em modo **Dual PWM (Slow-Decay / Freio Dinâmico Ativo)**, utilizando as macros Low-Layer (LL) da STMicroelectronics para máxima velocidade de processamento e chaveamento, além de possuir suporte nativo para segurança de concorrência (*thread-safety*).

### Recursos de Performance:
* **Resolução Dinâmica Nativa:** A entrada de velocidade (de `-100.0f` a `100.0f`) é mapeada diretamente para a contagem de ciclos do timer de hardware, aproveitando a precisão física máxima (sem passos lógicos intermediários limitados a 10 ou 12 bits).
* **Diagnóstico de Inicialização:** No boot, a função `engine_driver_init` imprime no console a frequência física em Hz e a resolução real em passos. Um aviso de atenção (`WARNING`) será exibido se a resolução for menor do que 1024 passos para garantir o controle ideal.

---

## 🔌 Pinagem Sugerida

Abaixo está o esquema de ligação elétrica recomendado entre o módulo **BTS7960 (IBT-2)** e a placa de desenvolvimento **WeAct STM32G431 Core Board**:

| Pino IBT-2 (Lado Controle) | Sinal | Pino STM32G431 | Descrição |
| :--- | :--- | :--- | :--- |
| **1 (RPWM)** | Entrada de sinal horário | **PA0** | Canal 1 do Timer `TIM2` (Chaveamento PWM) |
| **2 (LPWM)** | Entrada de sinal anti-horário | **PA1** | Canal 2 do Timer `TIM2` (Chaveamento PWM) |
| **3 (R_EN)** | Enable do sentido horário | **PA4** | GPIO de Habilitação (Ativa em HIGH) |
| **4 (L_EN)** | Enable do sentido anti-horário | **PA4** | GPIO de Habilitação (Ativa em HIGH, conectada com R_EN) |
| **5 (R_IS)** | Alarme de corrente horária | *Não conectado* | Saída analógica opcional para leitura de sobrecorrente |
| **6 (L_IS)** | Alarme de corrente anti-horária | *Não conectado* | Saída analógica opcional para leitura de sobrecorrente |
| **7 (VCC)** | Tensão lógica do buffer | **3.3V** | Alimenta a lógica do buffer de entrada do módulo (74HC244) |
| **8 (GND)** | Terra lógico comum | **GND** | Conexão de referência comum de terra (Obrigatório) |

> [!WARNING]
> **Compatibilidade Lógica (3.3V vs 5V):**
> O módulo IBT-2 possui um chip buffer de entrada CMOS (`74HC244`). Se você alimentar o pino **7 (VCC)** do módulo com 5V, o limiar mínimo para ele reconhecer o sinal alto será de $3.5\text{V}$, causando falha ou comportamento instável, já que as saídas do STM32 são de $3.3\text{V}$. 
> **Alimentar o pino VCC do controle do módulo com 3.3V** resolve esse problema nativamente, ajustando o limiar de leitura da ponte H para a tensão lógica do STM32.

| Pino IBT-2 (Potência) | Função | Conexão |
| :--- | :--- | :--- |
| **B+** | Alimentação positiva de potência | Terminal positivo da fonte/bateria do motor (6V a 27V DC) |
| **B-** | Terra de potência | Terminal negativo da fonte/bateria do motor |
| **M+ / R_OUT** | Saída positiva para motor | Terminal 1 do motor DC |
| **M- / L_OUT** | Saída negativa para motor | Terminal 2 do motor DC |

---

## ⚙️ Integração em Outros Projetos Zephyr

Para levar este driver para outro projeto Zephyr RTOS, siga os passos abaixo:

### Passo 1: Copiar a Pasta do Driver
Copie a pasta `engine-driver/` para o diretório de fontes do seu projeto (por exemplo, dentro de `src/engine-driver/`).

### Passo 2: Configurar o `CMakeLists.txt`
No arquivo `CMakeLists.txt` raiz do seu projeto novo, adicione a subpasta e vincule a biblioteca estática ao seu executável (`app`):
```cmake
add_subdirectory(src/engine-driver)
target_link_libraries(app PRIVATE engine_driver)
```

### Passo 3: Copiar as Definições do Devicetree
1. Copie o arquivo de template de binding `generic-engine.example.yml` (localizado dentro desta pasta) para o diretório de bindings do seu novo projeto, renomeando-o para `generic-engine.yaml` (geralmente sob `dts/bindings/generic-engine.yaml` ou `boards/bindings/generic-engine.yaml`).
2. Adicione as seguintes configurações ao seu arquivo de overlay da placa (ex: `app.overlay`):
   ```dts
   / {
       engine: engine {
           compatible = "generic-engine";
           pwms = <&pwm2 1 50000 PWM_POLARITY_NORMAL>, /* TIM2 CH1 no PA0 (50us = 20kHz) */
                  <&pwm2 2 50000 PWM_POLARITY_NORMAL>; /* TIM2 CH2 no PA1 (50us = 20kHz) */
           enable-gpios = <&gpioa 4 GPIO_ACTIVE_HIGH>; /* R_EN e L_EN no PA4 */
           status = "okay";
       };
   };

   &timers2 {
       status = "okay";
       pwm2: pwm {
           status = "okay";
           pinctrl-0 = <&tim2_ch1_pa0 &tim2_ch2_pa1>; /* Ativa pinagem PWM de hardware no PA0 e PA1 */
           pinctrl-names = "default";
       };
   };
   ```

### Passo 4: Configurar o `prj.conf` e `Kconfig`
No arquivo `prj.conf` do seu projeto novo, ative as flags requeridas:
```kconfig
# Habilita o subsistema de PWM do Zephyr
CONFIG_PWM=y

# Habilita Thread-Safe no acionamento do motor se necessário (opcional)
CONFIG_ENGINE_THREAD_SAFE=y
```
Se utilizar a flag de sincronização acima (`CONFIG_ENGINE_THREAD_SAFE`), lembre-se de declarar o menu de configuração correspondente em seu arquivo `Kconfig` no nível raiz do projeto.

---

## 💻 Exemplo de Uso

Aqui está um exemplo simples de código em C demonstrando como declarar, inicializar e controlar a velocidade do motor:

```c
#include <zephyr/kernel.h>
#include <stdio.h>
#include "engine_driver.h"

// Define a estrutura do driver com base nos nós criados no Devicetree
static struct engine_config engine = {
    .pwm_fwd = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(engine), 0),
    .pwm_rev = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(engine), 1),
    .enable  = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(engine), enable_gpios, {0}),
};

int main(void) {
    printf("Inicializando driver do motor...\n");
    
    int ret = engine_driver_init(&engine);
    if (ret < 0) {
        printf("Erro na inicialização do driver (%d)\n", ret);
        return ret;
    }
    
    printf("Inicializado com sucesso!\n");

    while (1) {
        // Gira no sentido horário a 30% de velocidade
        engine_driver_set_speed(&engine, 30.0f);
        k_msleep(3000);

        // Aplica freio eletrônico ativo (Slow Decay)
        engine_driver_set_speed(&engine, 0.0f);
        k_msleep(1500);

        // Gira no sentido anti-horário a 50% de velocidade
        engine_driver_set_speed(&engine, -50.0f);
        k_msleep(3000);

        // Para e aguarda
        engine_driver_set_speed(&engine, 0.0f);
        k_msleep(2000);
    }

    return 0;
}
```
---
![SmartSensing.me Logo](https://smartsensing.me/ssme-logo.png)

## 📝 Descrição

Este projeto faz parte do ecossistema **SmartSensing.me** e vai além dos exemplos básicos encontrados na internet. Aqui, aplicamos os fundamentos reais da engenharia de instrumentação e sistemas embarcados de alta performance.

Diferente de conteúdos superficiais voltados apenas para cliques, este repositório entrega:
- **Ineditismo:** Implementações originais baseadas em quase 30 anos de experiência acadêmica.
- **Densidade Técnica:** Uso profissional do framework ESP-IDF e FreeRTOS.
- **Didática:** Código documentado e estruturado para quem busca evolução técnica real.

> "Transformamos sinais do mundo físico em inteligência digital, sem atalhos."

---

## 🛠️ Tecnologias e Compatibilidade
- **Linguagem:** C puro (C99 ou superior) e C++
- **Hardware Alvo:** Qualquer microcontrolador (ESP32, STM32, ARM Cortex, RISC-V, AVR, etc.) ou arquitetura desktop
- **Ambientes/RTOS:** ESP-IDF (como Componente nativo), Zephyr RTOS, FreeRTOS, Bare-metal, Desktop (Windows, Linux, macOS)
- **Build System:** CMake nativo
- **Simulação:** LTSpice (Modelagem e validação de sensores)

---

## 👤 Sobre o Autor

**José Alexandre de França** *Professor Adjunto no Departamento de Engenharia Elétrica da UEL*

Engenheiro Eletricista com quase três décadas de experiência no ensino de graduação e pós-graduação. Doutor em Engenharia Elétrica, pesquisador em instrumentação eletrônica e desenvolvedor de sistemas embarcados. O SmartSensing.me é o meu compromisso de elevar o nível da educação tecnológica no Brasil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 Licença

Este projeto está sob a licença MIT. Veja o arquivo [LICENSE](LICENSE) para detalhes.
