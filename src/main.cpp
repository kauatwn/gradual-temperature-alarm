/*
 * Sistema de Monitorização e Alarme de Temperatura com ESP32
 *
 * Descrição do Projeto:
 * Firmware para monitoramento contínuo de temperatura utilizando sensor NTC, resposta visual gradual via PWM em LED e
 * alarme sonoro com buzzer piezoelétrico para condições críticas. O sistema avalia a temperatura em tempo real, modula
 * a intensidade do LED e transmite telemetria periodicamente via porta serial.
 *
 * Regras de Negócio e Comportamento Operacional:
 * 1. Temperatura e Processamento (Sensor NTC no pino GPIO 34):
 *    - Leitura analógica (0 a 4095) e conversão para graus Celsius (°C) via Equação Beta.
 *    - Abaixo de 25.0 °C (Normal): LED totalmente apagado (0%) e buzzer desligado.
 *    - De 25.0 °C a 60.0 °C (Alerta Proporcional): LED acende gradualmente via PWM (0 a 255).
 *    - Acima de 70.0 °C (Crítico / Alarme): LED em brilho máximo (100%), buzzer contínuo ativo e envio do aviso
 *    "! ALERTA: TEMPERATURA CRÍTICA !" via porta serial.
 * 2. Sinalização e Atuadores:
 *    - LED Vermelho (GPIO 23): Alerta visual proporcional (PWM) de aquecimento.
 *    - Buzzer (GPIO 19): Alarme sonoro contínuo (1000 Hz) ativado a partir de 70.0 °C.
 */

#include <Arduino.h>

namespace {
// Mapeamento de pinos do hardware
constexpr uint8_t pin_ntc = 34;     // Entrada analógica: sensor de temperatura (NTC)
constexpr uint8_t pin_led = 23;     // Saída PWM: LED de sinalização gradual e alerta
constexpr uint8_t pin_buzzer = 19;  // Saída digital: buzzer piezoelétrico

// Parâmetros do conversor analógico-digital (ADC de 12 bits do ESP32)
constexpr float adc_raw_max = 4095.0F;  // Resolução de 12 bits (2^12 - 1 = 4095)
constexpr int adc_raw_min = 0;          // Leitura mínima do ADC
constexpr int adc_raw_max_int = 4095;   // Leitura máxima do ADC

// Parâmetros do termistor NTC para a Equação Beta (B3950, R0 = 10 kOhm a 25 °C)
constexpr float ntc_beta = 3950.0F;
constexpr float ntc_t0_kelvin = 298.15F;
constexpr float absolute_zero_celsius = 273.15F;
constexpr float ntc_max_temp_c = 125.0F;  // Limite físico de saturação do NTC
constexpr float ntc_min_temp_c = -40.0F;  // Limite físico de saturação do NTC

// Limiares operacionais de temperatura (°C)
constexpr float temp_threshold_min = 25.0F;    // Normal: < 25.0 °C | Alerta Proporcional: >= 25.0 °C
constexpr float temp_threshold_max = 60.0F;    // Limite superior do alerta proporcional: 60.0 °C
constexpr float temp_threshold_alarm = 70.0F;  // Estado Crítico / Alarme sonoro: >= 70.0 °C

// Limites do PWM para o LED (resolução de 8 bits: 0 a 255)
constexpr uint8_t pwm_off = 0;         // PWM desligado (0% de duty cycle)
constexpr uint8_t pwm_max_duty = 255;  // Ciclo de trabalho máximo (100% de brilho)

// Temporizações, parâmetros acústicos e comunicação serial
constexpr unsigned long serial_baud_rate = 115200;     // Velocidade da porta serial (115200 bps)
constexpr unsigned long telemetry_interval_ms = 1000;  // Intervalo de transmissão serial (1 segundo)
constexpr unsigned long sampling_interval_ms = 100;    // Intervalo de amostragem do sensor (100 ms)
constexpr unsigned int buzzer_frequency_hz = 1000;     // Frequência do alarme sonoro no buzzer (1000 Hz)
constexpr uint8_t telemetry_decimals = 1;              // Casas decimais da temperatura na telemetria

// Classificação operacional das faixas de temperatura
enum class SystemState : uint8_t {
  Normal,             // Abaixo de 25.0 °C: operacao normal (LED apagado, Buzzer desligado)
  ProportionalAlert,  // 25.0 °C a 60.0 °C: alerta proporcional com PWM gradual no LED
  ElevatedWarning,    // Acima de 60.0 °C até 70.0 °C: aquecimento pré-crítico (LED 100%, Buzzer desligado)
  CriticalAlarm,      // A partir de 70.0 °C: estado crítico de emergência (LED 100%, Buzzer ativo)
};

// Pacote agregado de telemetria para transporte e transmissão serial
struct TemperatureTelemetry {
  float temperature_c;
  int raw_adc;
  uint8_t pwm_duty;
  float pwm_percentage;
  SystemState state;
  bool buzzer_active;
};

// Variáveis de estado global do sistema
auto current_state = SystemState::Normal;
unsigned long last_telemetry_ms = 0;
unsigned long last_sampling_ms = 0;
TemperatureTelemetry latest_telemetry = {
    .temperature_c = 0.0F,
    .raw_adc = 0,
    .pwm_duty = pwm_off,
    .pwm_percentage = 0.0F,
    .state = SystemState::Normal,
    .buzzer_active = false,
};

// Leitura analógica da porta do sensor NTC
int read_temperature_adc() { return analogRead(pin_ntc); }

// Conversão da leitura ADC para graus Celsius via Equação Beta
float calculate_temperature_celsius(const int raw_adc) {
  // Tratamento de limites físicos do NTC (evita divisões por zero)
  if (raw_adc <= adc_raw_min) {
    return ntc_max_temp_c;
  }
  if (raw_adc >= adc_raw_max_int) {
    return ntc_min_temp_c;
  }

  // Conversão via Equação do Parâmetro Beta (Steinhart-Hart simplificada)
  const float raw_ratio = adc_raw_max / static_cast<float>(raw_adc);
  const float adc_ratio = 1.0F / (raw_ratio - 1.0F);
  if (adc_ratio <= 0.0F) {
    return ntc_max_temp_c;
  }

  const float log_ratio = logf(adc_ratio);
  const float term_beta = log_ratio / ntc_beta;
  constexpr float term_t0 = 1.0F / ntc_t0_kelvin;
  const float kelvin = 1.0F / (term_beta + term_t0);

  return kelvin - absolute_zero_celsius;
}

// Classifica o estado operacional conforme a temperatura medida
SystemState classify_state(const float temp_c) {
  if (temp_c < temp_threshold_min) {
    return SystemState::Normal;
  }
  if (temp_c <= temp_threshold_max) {
    return SystemState::ProportionalAlert;
  }
  if (temp_c < temp_threshold_alarm) {
    return SystemState::ElevatedWarning;
  }
  return SystemState::CriticalAlarm;
}

// Interpolação linear do brilho do LED via PWM na faixa proporcional (25.0 °C a 60.0 °C)
uint8_t calculate_pwm_duty(const float temp_c) {
  if (temp_c <= temp_threshold_min) {
    return pwm_off;
  }
  if (temp_c >= temp_threshold_max) {
    return pwm_max_duty;
  }

  const float ratio = (temp_c - temp_threshold_min) / (temp_threshold_max - temp_threshold_min);
  const float calculated_duty = ratio * static_cast<float>(pwm_max_duty);
  const int duty_int = static_cast<int>(roundf(calculated_duty));

  if (duty_int < pwm_off) {
    return pwm_off;
  }
  if (duty_int > pwm_max_duty) {
    return pwm_max_duty;
  }
  return static_cast<uint8_t>(duty_int);
}

// Conversão do ciclo de trabalho do PWM para porcentagem (0.0% a 100.0%)
float calculate_pwm_percentage(const uint8_t duty) {
  return static_cast<float>(duty) / static_cast<float>(pwm_max_duty) * 100.0F;
}

// Atualização da intensidade do LED via PWM (apenas se houver alteração)
void update_visual_signaling(const uint8_t duty) {
  static int last_duty = -1;
  if (duty != last_duty) {
    analogWrite(pin_led, duty);
    last_duty = duty;
  }
}

// Acionamento do alarme sonoro (apenas em transição de estado para evitar reiniciar o oscilador)
void update_acoustic_alarm(const bool activate) {
  static bool buzzer_is_active = false;

  // Se o estado for igual ao atual, não há nada a fazer (aborta a função)
  if (activate == buzzer_is_active) {
    return;
  }

  // Atualiza a memória estática
  buzzer_is_active = activate;

  // Aplica a mudança física no hardware
  if (activate) {
    tone(pin_buzzer, buzzer_frequency_hz);
  } else {
    noTone(pin_buzzer);
  }
}

// Atualização dos atuadores físicos (LED e Buzzer)
void update_actuators(const SystemState state, const uint8_t duty) {
  update_visual_signaling(duty);
  update_acoustic_alarm(state == SystemState::CriticalAlarm);
}

// Retorna o rótulo textual do status do sistema para a telemetria serial
const __FlashStringHelper* get_state_label(const SystemState state) {
  switch (state) {
    case SystemState::Normal:
      return F("NORMAL");
    case SystemState::ProportionalAlert:
      return F("ALERTA PROPORCIONAL (PWM)");
    case SystemState::ElevatedWarning:
      return F("AQUECIMENTO ELEVADO (LED 100%)");
    case SystemState::CriticalAlarm:
      return F("CRÍTICO / ALARME");
  }
  return F("INDEFINIDO");
}

// Emite mensagem de emergência na porta serial
void notify_critical_alarm() { Serial.println(F("! ALERTA: TEMPERATURA CRÍTICA !")); }

// Transmissão periódica das informações pela porta serial a partir do pacote de telemetria
void transmit_telemetry(const TemperatureTelemetry& telemetry) {
  Serial.print(F("[TELEMETRIA] Temp: "));
  Serial.print(telemetry.temperature_c, telemetry_decimals);
  Serial.print(F(" °C | ADC: "));
  Serial.print(telemetry.raw_adc);

  Serial.print(F(" | LED PWM: "));
  Serial.print(telemetry.pwm_duty);
  Serial.print(F(" ("));
  Serial.print(telemetry.pwm_percentage, 1);
  Serial.print(F("%) | Buzzer: "));
  Serial.print(telemetry.buzzer_active ? F("ATIVO!") : F("Inativo"));

  Serial.print(F(" | Status: "));
  Serial.println(get_state_label(telemetry.state));
}
}  // namespace

void setup() {
  Serial.begin(serial_baud_rate);
  Serial.println(F("=================================================="));
  Serial.println(F(" SISTEMA DE MONITORIZAÇÃO DE TEMPERATURA - ESP32 "));
  Serial.println(F(" Status: Inicializado com Sucesso                 "));
  Serial.println(F("=================================================="));

  pinMode(pin_ntc, INPUT);
  pinMode(pin_led, OUTPUT);
  pinMode(pin_buzzer, OUTPUT);

  // Amostragem inicial e definição do estado de partida seguro
  const int initial_adc = read_temperature_adc();
  const float initial_temp = calculate_temperature_celsius(initial_adc);
  current_state = classify_state(initial_temp);
  const uint8_t initial_duty = calculate_pwm_duty(initial_temp);

  latest_telemetry = {
      .temperature_c = initial_temp,
      .raw_adc = initial_adc,
      .pwm_duty = initial_duty,
      .pwm_percentage = calculate_pwm_percentage(initial_duty),
      .state = current_state,
      .buzzer_active = current_state == SystemState::CriticalAlarm,
  };

  update_actuators(current_state, initial_duty);
}

void loop() {
  const unsigned long current_ms = millis();

  // Amostragem periódica do sensor a cada 100 ms (elimina sobrecarga da CPU)
  if (current_ms - last_sampling_ms >= sampling_interval_ms) {
    last_sampling_ms = current_ms;

    const int raw_adc = read_temperature_adc();
    const float temp_c = calculate_temperature_celsius(raw_adc);

    const SystemState new_state = classify_state(temp_c);
    const uint8_t pwm_duty = calculate_pwm_duty(temp_c);
    const float pwm_pct = calculate_pwm_percentage(pwm_duty);

    // Atualização imediata dos atuadores
    update_actuators(new_state, pwm_duty);

    // Notificação imediata em transição para o estado crítico
    if (new_state == SystemState::CriticalAlarm && current_state != SystemState::CriticalAlarm) {
      notify_critical_alarm();
    }

    latest_telemetry = {
        .temperature_c = temp_c,
        .raw_adc = raw_adc,
        .pwm_duty = pwm_duty,
        .pwm_percentage = pwm_pct,
        .state = new_state,
        .buzzer_active = new_state == SystemState::CriticalAlarm,
    };

    current_state = new_state;
  }

  // Transmissão periódica da telemetria serial a cada 1 segundo (1000 ms)
  if (current_ms - last_telemetry_ms >= telemetry_interval_ms) {
    last_telemetry_ms = current_ms;

    if (current_state == SystemState::CriticalAlarm) {
      notify_critical_alarm();
    }
    transmit_telemetry(latest_telemetry);
  }
}
