# Driver de Motor Duplo PWM BTS7960 (IBT-2) para ESP-IDF

*Read in other languages: [English](README.md)*

Este módulo fornece um driver C estático modular e otimizado para controle de motores DC usando a ponte H de alta corrente **BTS7960** (comumente vendida como placa de interface **IBT-2**) no **ESP32-S3** sob o **ESP-IDF**.

O controle é feito em modo **Dual PWM (Slow-Decay / Frenagem Dinâmica Ativa)**, utilizando o periférico nativo **MCPWM** do ESP32-S3 para máxima resolução e velocidade de comutação, com suporte nativo a thread-safety.

### Principais Características de Desempenho:
* **Alta Resolução:** O timer é configurado com uma resolução de clock de 80 MHz (frequência física máxima do timer no ESP32-S3), resultando em exatamente **4000 passos de resolução** para uma frequência de 20 kHz.
* **Dead-time Configurável por Software:** Protege a ponte H durante inversões de sentido com um intervalo configurável em COAST (50 us por padrão).
* **Thread-safety Selecionável:** A segurança de threads é configurável em tempo de compilação usando `CONFIG_ENGINE_THREAD_SAFE` via Kconfig/menuconfig.
* **Controle cache-safe opcional:** O caminho de comando da ponte pode ser
  colocado em IRAM junto com as rotinas de controle MCPWM/GPIO do ESP-IDF.

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
| **6 (L_IS)** | Corrente sentido anti-horário | **GPIO 5 condicionado** | Diagnóstico de corrente reversa e falha |
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
* **`CONFIG_ENGINE_CACHE_SAFE_CONTROL`:** Coloca `set_speed`, `coast`,
  `brake`, leitura de estado e toda a cadeia interna correspondente em IRAM.
  Exige thread-safety desabilitado e seleciona automaticamente as opções
  cache-safe de MCPWM/GPIO do ESP-IDF (Padrão: `n`).
* **`CONFIG_ENGINE_PWM_FREQ_HZ`:** Frequência do sinal PWM em Hz (Padrão: `20000` / 20 kHz).
* **`CONFIG_ENGINE_PIN_RPWM`:** Número do GPIO usado para o controle Forward (Padrão: `1`).
* **`CONFIG_ENGINE_PIN_LPWM`:** Número do GPIO usado para o controle Reverse (Padrão: `2`).
* **`CONFIG_ENGINE_PIN_ENABLE`:** GPIO ligado conjuntamente a R_EN/L_EN (Padrão: `3`). Ele é obrigatório para implementar o estado `COAST`.
* **`CONFIG_ENGINE_DIRECTION_DEAD_TIME_US`:** Intervalo em COAST antes de inverter o acionamento (Padrão: `50` us).
* **`CONFIG_ENGINE_CURRENT_SENSE_ENABLE`:** Habilita a aquisição contínua de `R_IS` e `L_IS` por ADC1/DMA.
* **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_R_IS`:** Entrada ADC1 condicionada (Padrão: `4`).
* **`CONFIG_ENGINE_CURRENT_SENSE_GPIO_L_IS`:** Entrada ADC1 condicionada independente (Padrão: `5`).
* **`CONFIG_ENGINE_CURRENT_SENSE_SAMPLE_HZ`:** Taxa por canal (Padrão: `25000`), formando 25 amostras de cada entrada por frame de 1 ms.
* **`CONFIG_ENGINE_CURRENT_SENSE_FAULT_ENTER_MV`:** Limiar calibrado de entrada no estado de falha de `I_IS` (Padrão: `1500` mV).
* **`CONFIG_ENGINE_CURRENT_SENSE_FAULT_EXIT_MV`:** Limiar de saída com histerese (Padrão: `1000` mV).

### Medição opcional de corrente

A placa usada neste projeto possui 10 kΩ entre cada saída `I_IS` e GND. Nenhum
GPIO deve ser ligado diretamente e `R_IS` não deve ser unido a `L_IS`. Para
cada canal, adicione seu próprio resistor de 10 kΩ em série até o ADC, 1 kΩ do
nó ADC para GND, 100 nF do nó ADC para GND e clamps Schottky externos para
3,3 V/GND.

O ADC contínuo roda no Core 0 e não realiza conversões bloqueantes na tarefa de
controle. Cada frame de 1 ms contém 25 amostras de cada canal. A histerese de
falha e os diagnósticos são independentes para `R_IS` e `L_IS`. A tarefa publica
as duas magnitudes calibradas, uma máscara de validade e uma máscara de falha.
A aplicação escolhe `R_IS` durante acionamento positivo e `L_IS` durante
acionamento negativo. Assim, ruído no canal inativo não troca indevidamente o
sinal da corrente.

A tarefa do ADC calibra as duas médias normais do último frame e publica ambas
em miliampères inteiros por `engine_current_sense_get_latest_measurement()`; a
calibração ADC nunca é chamada pelo caminho de controle. A aplicação conserva
os dois canais na estrutura publicada e usa o estado efetivo do driver para
determinar a direção da corrente sem descartar informação.

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

### Caminho de controle com cache desabilitado

Habilite `CONFIG_ENGINE_CACHE_SAFE_CONTROL` apenas quando uma única tarefa
determinística for proprietária da instância. Além de mover os comandos do
driver para IRAM, a opção seleciona `CONFIG_MCPWM_CTRL_FUNC_IN_IRAM` e
`CONFIG_GPIO_CTRL_FUNC_IN_IRAM`; a opção do MCPWM também impede que seus objetos
de execução sejam alocados em RAM externa. Um fragmento condicional do linker
também move `mcpwm_generator_set_force_level()`, pois o ESP-IDF 6.0 não inclui
essa operação no mapeamento IRAM das funções de controle. A
`struct engine_config` fornecida pelo chamador, a pilha e o restante da cadeia
da aplicação ainda devem residir em memória interna.

Inicialização, logs, aquisição de corrente e seus relatórios ficam fora dessa
garantia. Uma inversão de sentido também pode executar o tempo morto bloqueante
configurado; portanto, ser cache-safe não garante por si só um prazo específico.

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
    .direction_dead_time_us = CONFIG_ENGINE_DIRECTION_DEAD_TIME_US,
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

## O comando representa tensão, não velocidade

Apesar do nome histórico `engine_driver_set_speed()`, o argumento é uma razão
cíclica assinada, não uma velocidade medida ou garantida:

```text
comando > 0  -> PWM em RPWM, LPWM baixo
comando < 0  -> PWM em LPWM, RPWM baixo
comando = 0  -> COAST
```

O módulo limita o comando a `[-100, +100] %`. A velocidade resultante depende
da fonte, motor, carga, atrito e força contraeletromotriz. Uma malha de
velocidade deve medir a rotação e produzir esse comando por meio de um
controlador, como o componente `esp_pid`.

## Estados elétricos da ponte

| Estado da API | R_EN/L_EN | RPWM | LPWM | Comportamento esperado |
|---|---:|---:|---:|---|
| `COAST` | 0 | 0 forçado | 0 forçado | Ponte desabilitada; eixo livre |
| `BRAKE` | 1 | 0 forçado | 0 forçado | Frenagem ativa pelos transistores inferiores |
| `DRIVE`, comando positivo | 1 | PWM | 0 forçado | Torque no sentido positivo |
| `DRIVE`, comando negativo | 1 | 0 forçado | PWM | Torque no sentido negativo |

`COAST` e `BRAKE` não são equivalentes. Em `COAST`, a corrente do motor decai
sem uma trajetória de frenagem ativa comandada. Em `BRAKE`, os terminais do
motor são colocados no estado de frenagem baixa do módulo, convertendo energia
mecânica principalmente em perdas no motor e na ponte.

O comportamento exato e a segurança elétrica dependem da placa IBT-2 real, da
fonte, do motor e de módulos clonados. O driver não mede tensão do barramento,
temperatura de junção nem energia regenerada.

## Inversão segura de sentido

Quando o motor já está em `DRIVE` e o sinal de um comando não nulo muda, o
driver executa:

```text
DRIVE atual
   -> força RPWM e LPWM em zero
   -> desabilita R_EN/L_EN: COAST
   -> espera direction_dead_time_us
   -> configura o comparador do sentido oposto
   -> habilita R_EN/L_EN com ambas as entradas baixas
   -> libera somente o PWM selecionado
```

O intervalo reduz o risco de comandar sentidos opostos durante uma transição,
mas não limita corrente, torque ou energia. A espera usa `esp_rom_delay_us()` e
é bloqueante; portanto, não escolha um tempo excessivo dentro de uma tarefa de
controle em tempo real.

## Ciclo de vida do driver

1. Preencha apenas os campos de configuração de `struct engine_config`.
2. Chame `engine_driver_init()` uma vez.
3. Aplique comandos com `engine_driver_set_speed()`, `engine_driver_brake()` ou
   `engine_driver_coast()`.
4. Consulte o modo efetivo com `engine_driver_get_state()` quando necessário.

A estrutura deve permanecer válida durante toda a operação, pois também contém
os handles MCPWM e o estado interno. O componente atual não oferece `deinit()`;
os recursos MCPWM permanecem pertencentes à instância durante a vida da
aplicação.

## Referência da API de acionamento

### `engine_driver_init()`

Valida a configuração, configura o enable, cria timer, operador, comparadores e
geradores MCPWM, força ambas as entradas em zero, inicia o timer, cria o mutex
opcional e termina em `COAST`. Retorna `0` em sucesso e `-1` em falha. Essa API
histórica não devolve o `esp_err_t` específico da etapa que falhou.

É chamada apenas pela aplicação. Internamente usa `force_both_pwm_low()` e
`engine_driver_coast()`.

### `engine_driver_set_speed()`

Aplica razão cíclica assinada e saturada. Um zero exato seleciona `COAST`. Em
uma inversão, chama `coast_locked()`, aguarda o tempo morto e configura o canal
oposto. Em erro de GPIO/MCPWM, tenta retornar para `COAST`.

Não passe `NaN`; as comparações e a conversão para ticks exigem um número
finito. A função retorna `ESP_OK`, `ESP_ERR_INVALID_ARG` para instância nula ou
o primeiro erro de periférico.

### `engine_driver_coast()`

Força RPWM e LPWM baixos e desabilita o GPIO comum de enable. A implementação
de `COAST` exige `pin_enable >= 0`; sem esse pino, desabilitar retorna
`ESP_ERR_NOT_SUPPORTED`.

### `engine_driver_brake()`

Força os dois PWM baixos e mantém o enable ativo, selecionando a frenagem ativa
low-side prevista para este módulo.

### `engine_driver_get_state()`

Copia o modo aplicado e a direção efetiva sem acessar os periféricos:

- `direction = +1`: `DRIVE` positivo;
- `direction = -1`: `DRIVE` negativo;
- `direction = 0`: `COAST` ou `BRAKE`.

Use o par `mode`/`direction` para interpretar o canal de corrente correto.

## Aquisição de corrente: arquitetura

Quando `CONFIG_ENGINE_CURRENT_SENSE_ENABLE=y`, `engine_current_sense_start()`
configura o ADC1 contínuo e o DMA para alternar `R_IS` e `L_IS`. Uma callback de
interrupção apenas acorda uma tarefa de baixa prioridade no Core 0. Essa tarefa:

1. drena e decodifica os frames DMA;
2. separa resultados de `R_IS` e `L_IS`;
3. aplica a histerese de falha de cada canal;
4. exclui amostras de falha da média de corrente normal;
5. calibra as médias com handles independentes do ADC;
6. publica uma estrutura coerente por frame;
7. publica também a interface assinada legada sem bloquear o controle.

O caminho de controle apenas lê dados já processados; ele não executa conversão
ADC nem calibração em ponto flutuante.

## Condicionamento de cada `I_IS`

Não una `R_IS` e `L_IS`. Cada saída precisa de sua própria rede:

```text
I_IS do módulo ---- 10 kΩ série ----+---- GPIO ADC1
                                    |
                                    +---- 1 kΩ ---- GND
                                    |
                                    +---- 100 nF -- GND
                                    |
                                    +---- clamps Schottky para 3V3 e GND
```

A placa considerada possui ainda 10 kΩ de `I_IS` para GND. Esses três valores
devem corresponder às opções `BOARD_RESISTOR`, `SERIES_RESISTOR` e
`PULLDOWN_RESISTOR`. O capacitor atenua comutação e ruído, mas não substitui os
resistores nem os clamps de proteção.

Com os valores padrão, o código considera o carregamento em paralelo causado
pela rede externa, calcula a transimpedância efetiva no ADC e multiplica a
corrente de sense pelo `k_ILIS` nominal. Como esse parâmetro possui tolerância
larga, a leitura deve ser comparada com um amperímetro de referência antes de
ser usada como proteção quantitativa.

## Corrente, direção e falha

`engine_current_sense_get_latest_measurement()` retorna magnitudes não negativas
separadas:

```c
engine_current_sense_measurement_t measurement;
engine_driver_state_t state;

if (engine_current_sense_get_latest_measurement(&measurement) &&
    engine_driver_get_state(&motor, &state) == ESP_OK &&
    state.mode == ENGINE_DRIVER_MODE_DRIVE) {
    engine_current_sense_channel_t active =
        state.direction > 0 ? ENGINE_CURRENT_SENSE_CHANNEL_R_IS
                            : ENGINE_CURRENT_SENSE_CHANNEL_L_IS;
    uint32_t bit = ENGINE_CURRENT_SENSE_CHANNEL_BIT(active);
    if ((measurement.valid_mask & bit) != 0U &&
        (measurement.fault_mask & bit) == 0U) {
        int32_t signed_ma = measurement.channel_milliamps[active] *
                            state.direction;
        /* signed_ma now follows the effective bridge direction. */
    }
}
```

O pino `I_IS` multiplexa medição de corrente e indicação de falha. Uma tensão
acima de `FAULT_ENTER_MV` ativa o estado de falha; ele só termina abaixo de
`FAULT_EXIT_MV`. Essa histerese reduz alternâncias espúrias perto do limiar.
Uma indicação não identifica sozinha a causa física: sobrecorrente,
subtensão/sobretensão, temperatura ou outra proteção interna devem ser
correlacionadas com medições adicionais e com o datasheet.

## APIs de corrente

| Função | Consome dados? | Finalidade |
|---|:---:|---|
| `engine_current_sense_start` | — | Cria ADC/DMA, calibração, tarefa e callbacks |
| `engine_current_sense_stop` | — | Para e libera os recursos de aquisição |
| `engine_current_sense_get_latest_measurement` | Não | Correntes calibradas e máscaras de ambos os canais |
| `engine_current_sense_get_latest_dual_frame` | Não | Último frame bruto coerente de ambos os canais |
| `engine_current_sense_take_dual_snapshot` | Sim | Copia e zera a janela de diagnóstico dos dois canais |
| `engine_current_sense_channel_raw_to_millivolts` | Não | Converte raw usando a calibração do canal indicado |
| `engine_current_sense_adc_to_r_is_millivolts` | Não | Reconstrói a tensão no pino do módulo |
| `engine_current_sense_adc_to_i_is_milliamperes` | Não | Estima a corrente produzida por I_IS |
| `engine_current_sense_adc_to_amperes` | Não | Estima a magnitude da corrente do motor |

As funções `take_*snapshot()` zeram os acumuladores; duas tarefas consumidoras
dividiriam a janela entre si. Os getters `get_latest_*()` não consomem dados e
podem ler novamente o último frame publicado.

As descrições completas de parâmetros, retornos e relações de chamada estão em
[`engine_driver.h`](engine_driver.h) e
[`engine_current_sense.h`](engine_current_sense.h). As implementações estão
divididas em blocos funcionais comentados.

## Histórico de alterações incompatíveis

### 2026-09-10 — remoção das APIs de corrente de um único canal

Foram retiradas quatro funções mantidas anteriormente para compatibilidade com
a aquisição exclusiva de `R_IS`:

| Função retirada | Substituição |
|---|---|
| `engine_current_sense_take_snapshot()` | `engine_current_sense_take_dual_snapshot()` |
| `engine_current_sense_get_latest_frame()` | `engine_current_sense_get_latest_dual_frame()` |
| `engine_current_sense_get_latest_current_milliamps()` | `engine_current_sense_get_latest_measurement()` combinado com `engine_driver_get_state()` |
| `engine_current_sense_raw_to_millivolts()` | `engine_current_sense_channel_raw_to_millivolts()` |

Esta mudança é incompatível em nível de código-fonte. Ela elimina interfaces
que descartavam um dos canais ou inferiam o sinal sem considerar explicitamente
o estado aplicado à ponte.

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
