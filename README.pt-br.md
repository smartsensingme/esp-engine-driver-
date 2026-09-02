# Driver de Motor Duplo PWM BTS7960 (IBT-2) para ESP-IDF

*Read in other languages: [English](README.md)*

Este módulo fornece um driver C estático modular e otimizado para controle de motores DC usando a ponte H de alta corrente **BTS7960** (comumente vendida como placa de interface **IBT-2**) no **ESP32-S3** sob o **ESP-IDF**.

O controle é feito em modo **Dual PWM (Slow-Decay / Frenagem Dinâmica Ativa)**, utilizando o periférico nativo **MCPWM** do ESP32-S3 para máxima resolução e velocidade de comutação, com suporte nativo a thread-safety.

### Principais Características de Desempenho:
* **Alta Resolução:** O timer é configurado com uma resolução de clock de 80 MHz (frequência física máxima do timer no ESP32-S3), resultando em exatamente **4000 passos de resolução** para uma frequência de 20 kHz.
* **Dead-time por Software:** Protege a ponte H contra curto-circuito (shoot-through) durante inversões de sentido com um atraso de transição preciso de 50 us (`esp_rom_delay_us(50)`).
* **Thread-safety Selecionável:** A segurança de threads é configurável em tempo de compilação usando `CONFIG_ENGINE_THREAD_SAFE` via Kconfig/menuconfig.

---

## 🔌 Pinagem Sugerida (ESP32-S3)

Abaixo está o diagrama de fiação elétrica recomendado entre o módulo **BTS7960 (IBT-2)** e o **ESP32-S3 DevKit**:

| Pino IBT-2 (Controle) | Sinal | Pino ESP32-S3 | Descrição |
| :--- | :--- | :--- | :--- |
| **1 (RPWM)** | Entrada PWM Horário | **GPIO 1** | Gerador MCPWM 0A |
| **2 (LPWM)** | Entrada PWM Anti-horário | **GPIO 2** | Gerador MCPWM 0B |
| **3 (R_EN)** | Enable Horário | **GPIO 3** | GPIO de Enable (Ativo em HIGH, interligado ao L_EN) |
| **4 (L_EN)** | Enable Anti-horário | **GPIO 3** | GPIO de Enable (Ativo em HIGH, interligado ao R_EN) |
| **5 (R_IS)** | Corrente sentido horário | **GPIO 4 condicionado** | Saída analógica opcional de diagnóstico/corrente |
| **6 (L_IS)** | Alarme corrente anti-horário | *Not conectado* | Saída analógica opcional para leitura de sobrecorrente |
| **7 (VCC)** | Tensão lógica do buffer | **3.3V** | Alimenta a lógica do buffer de entrada do módulo (74HC244) |
| **8 (GND)** | Terra lógico comum | **GND** | Conexão comum de referência de terra (Obrigatório) |

> [!WARNING]
> **Compatibilidade Lógica (3.3V vs 5V):**
> O módulo IBT-2 possui um chip de buffer de entrada CMOS (`74HC244`). Se você alimentar o pino **7 (VCC)** do módulo com 5V, o limite mínimo para reconhecer um sinal em nível HIGH é de $3.5\text{V}$, causando falha ou comportamento instável, já que as saídas do ESP32-S3 são de $3.3\text{V}$. 
> **Alimentar o pino VCC lógica com 3.3V** resolve esse problema nativamente, ajustando o limite de leitura da ponte H para a tensão lógica do ESP32-S3.

| Pino IBT-2 (Potência) | Função | Conexão |
| :--- | :--- | :--- |
| **B+** | Alimentação positiva | Terminal positivo da fonte/bateria do motor (6V a 27V DC) |
| **B-** | Terra de potência | Terminal negativo da fonte/bateria do motor |
| **M+ / R_OUT** | Saída positiva motor | Terminal 1 do motor DC |
| **M- / L_OUT** | Saída negativa motor | Terminal 2 do motor DC |

---

## ⚙️ Configuração (menuconfig)

Você pode configurar graficamente os pinos, a frequência do PWM e o thread-safety executando:
```bash
idf.py menuconfig
```
Acesse **Component config** -> **Engine Driver Configuration**:
* **`CONFIG_ENGINE_THREAD_SAFE`:** Ativar proteção por Mutex (Padrão: `y`). Se desativado, todas as operações de mutex são removidas da compilação para máxima velocidade de processamento.
* **`CONFIG_ENGINE_PWM_FREQ_HZ`:** Frequência do sinal PWM em Hz (Padrão: `20000` / 20 kHz).
* **`CONFIG_ENGINE_PIN_RPWM`:** Número do GPIO usado para o controle Forward (Padrão: `1`).
* **`CONFIG_ENGINE_PIN_LPWM`:** Número do GPIO usado para o controle Reverse (Padrão: `2`).
* **`CONFIG_ENGINE_PIN_ENABLE`:** GPIO ligado conjuntamente a R_EN/L_EN (Padrão: `3`). Ele é obrigatório para implementar o estado `COAST`.
* **`CONFIG_ENGINE_CURRENT_SENSE_ENABLE`:** Habilita a aquisição contínua de `R_IS` por ADC1/DMA.
* **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS`:** Entrada ADC1 condicionada (Padrão: `4`).
* **`CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ`:** Taxa agregada (Padrão: `25000`), formando 25 amostras por frame de 1 ms.
* **`CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV`:** Limiar calibrado de entrada no estado de falha de `I_IS` (Padrão: `1500` mV).
* **`CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV`:** Limiar de saída com histerese (Padrão: `1000` mV).

### Medição opcional de corrente

A placa usada neste projeto possui 10 kΩ entre `R_IS` e GND. O GPIO não deve
ser ligado diretamente ao pino. Adicione 10 kΩ em série até o ADC, 1 kΩ do ADC
para GND, 100 nF do ADC para GND e clamps Schottky externos para 3,3 V/GND.

O ADC contínuo roda no Core 0 e não realiza conversões bloqueantes na tarefa de
controle. Cada frame de 1 ms fornece média e mediana das 25 amostras. A média é
a estimativa principal da corrente equivalente; a mediana é complementar e
ajuda a identificar impulsos ou outliers. Em duty baixo, a mediana pode ser
zero mesmo quando a corrente média não é zero.

O pino `I_IS` também sinaliza falhas do BTS7960 por uma corrente praticamente
independente da corrente da carga. Amostras acima do limiar configurado são
contadas como falha e não são convertidas em ampères. Os snapshots informam a
fração de amostras em falha, o número de entradas nesse estado e se ele continua
ativo ao final da janela.

Com os resistores 10 kΩ/10 kΩ/1 kΩ e `k_ILIS=8500`, a sensibilidade nominal no
ADC é aproximadamente 56 mV/A. A tolerância de `k_ILIS` exige calibração contra
um amperímetro confiável.

### Estados da ponte

`engine_driver_set_speed()` aplica PWM para comandos diferentes de zero. Um
comando exatamente zero seleciona `COAST`: força RPWM/LPWM em nível baixo e
desabilita R_EN/L_EN. `engine_driver_brake()` oferece frenagem ativa explícita,
mantendo os enables ativos com os dois PWM baixos. Os níveis estáticos usam a
força de saída do MCPWM, sem depender do caso ambíguo de compare igual a zero.

---

## 💻 Exemplo de Uso

```c
#include "esp_log.h"
#include "engine_driver.h"

static const char *TAG = "APP";

// Configuração estática usando os padrões do Kconfig
static struct engine_config motor = {
    .pin_fwd = CONFIG_ENGINE_PIN_RPWM,
    .pin_rev = CONFIG_ENGINE_PIN_LPWM,
    .pin_enable = CONFIG_ENGINE_PIN_ENABLE,
    .pwm_freq_hz = CONFIG_ENGINE_PWM_FREQ_HZ,
};

void app_main(void) {
    ESP_LOGI(TAG, "Inicializando driver do motor...");
    if (engine_driver_init(&motor) == 0) {
        ESP_LOGI(TAG, "Driver do motor inicializado com sucesso.");
        
        // Gira no sentido horário com 50% de velocidade
        engine_driver_set_speed(&motor, 50.0f);
    } else {
        ESP_LOGE(TAG, "Falha ao inicializar o driver do motor!");
    }
}
```

---

![SmartSensing.me Logo](https://smartsensing.me/ssme-logo.png)

## 📝 Descrição

Este projeto faz parte do ecossistema **SmartSensing.me** e vai além dos exemplos básicos encontrados na internet. Aqui, aplicamos os fundamentos reais da engenharia de instrumentação e sistemas embarcados de alta performance.

Diferente de conteúdos superficiais voltados apenas para cliques, este repositório entrega:
- **Originalidade:** Implementações originais baseadas em quase 30 anos de experiência acadêmica.
- **Densidade Técnica:** Uso profissional das frameworks ESP-IDF, Zephyr RTOS e FreeRTOS.
- **Didática:** Códigos documentados e estruturados para quem busca real crescimento técnico.

> "Transformamos sinais do mundo físico em inteligência digital, sem atalhos."

---

## 🛠️ Tecnologias e Compatibilidade
- **Linguagem:** C puro (C99 ou superior) e C++
- **Hardware Alvo:** ESP32-S3 (e outras SoCs da família ESP32 com suporte a MCPWM)
- **Ambientes/RTOS:** ESP-IDF (como Componente nativo)
- **Build System:** CMake Nativo

---

## 👤 Sobre o Autor

**José Alexandre de França** *Professor Associado no Departamento de Engenharia Elétrica da UEL*

Engenheiro Eletricista com quase três décadas de experiência na docência de graduação e pós-graduação. Doutor em Engenharia Elétrica, pesquisador em instrumentação eletrônica e desenvolvedor de sistemas embarcados. O SmartSensing.me é o meu compromisso com a elevação do nível da educação tecnológica no Brasil.

- 🌐 **Website:** [smartsensing.me](https://smartsensing.me)
- 📧 **E-mail:** [info@smartsensing.me](mailto:info@smartsensing.me)
- 📺 **YouTube:** [@smartsensingme](https://youtube.com/@smartsensingme)
- 📸 **Instagram:** [@smartsensing.me](https://instagram.com/smartsensing.me)

---

## 📄 Licença

Este projeto é licenciado sob a Licença MIT. Veja o arquivo [LICENSE](LICENSE) para detalhes.
