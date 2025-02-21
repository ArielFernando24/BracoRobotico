#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"


void desenhar_string_centralizado(ssd1306_t *ssd, const char *str, uint8_t y);
void draw_border(void);
void draw_square(ssd1306_t *ssd, int x, int y, int size, bool color);
void draw_pixel(ssd1306_t *ssd, int x, int y, bool color);

// Definições dos pinos e configurações
#define LED_RED_PIN     13  // Pino do LED vermelho
#define LED_GREEN_PIN   11  // Pino do LED verde
#define LED_BLUE_PIN    12  // Pino do LED azul
#define JOYSTICK_X_PIN  26  // Pino do eixo X do joystick (ADC)
#define JOYSTICK_Y_PIN  27  // Pino do eixo Y do joystick (ADC)
#define JOYSTICK_PB     22  // Botão do joystick para confirmar altura e outras funções
#define BUTTON_A_PIN    5   // Botão A para decrementar a altura
#define BUTTON_B_PIN    6   // Botão B para incrementar a altura
#define I2C_SDA         14  // Pino SDA do I2C
#define I2C_SCL         15  // Pino SCL do I2C
#define I2C_PORT        i2c1 // Porta I2C utilizada
#define SSD1306_ADDRESS 0x3C // Endereço I2C do display OLED
#define DEADZONE_X      10  // Zona morta para o eixo X do joystick
#define DEADZONE_Y      10  // Zona morta para o eixo Y do joystick
#define TARGET_TOLERANCE 0  // Tolerância para captura (em pixels)
#define DISPLAY_WIDTH   128 // Largura do display OLED
#define DISPLAY_HEIGHT  64  // Altura do display OLED
#define SQUARE_SIZE     8   // Tamanho do quadrado do cursor

// Tempo de exibição da tela de coordenadas (em ms)
#define TEMPO_EXIBICAO_ALVO 3000

// Variáveis globais
ssd1306_t display; // Estrutura do display OLED
bool pwm_enabled = true; // Habilita/desabilita o PWM dos LEDs
uint8_t border_style = 0; // Estilo da borda do display
uint16_t adc_value_x, adc_value_y; // Valores lidos do ADC (joystick)
int altura = 0;             // Altura atual (0-100)
int alvo_x = 0;             // Coordenada X do alvo
int alvo_y = 0;             // Coordenada Y do alvo
int alvo_altura = 0;        // Altura desejada para o alvo
bool target_acquired = false; // Indica se o alvo foi capturado
bool capture_mode = false;  // Modo de captura ativado/desativado
bool altura_ok = false;     // Confirmação da altura pelo usuário
bool local_ok = false;      // Confirmação da localização pelo usuário
bool in_position = false;   // Indica se o alvo está dentro da área do cursor
int cursor_x = 0;           // Posição X do cursor
int cursor_y = 0;           // Posição Y do cursor

// Variáveis para exibição temporária do alvo
bool exibir_alvo = false;   // Indica se deve exibir as coordenadas do alvo
uint32_t exibir_alvo_start_time = 0; // Tempo inicial da exibição do alvo

// Variáveis de debounce
static uint32_t last_time_joystick = 0; // Último tempo do botão do joystick
static uint32_t last_time_buttonA = 0;  // Último tempo do botão A
static uint32_t last_time_buttonB = 0;  // Último tempo do botão B

// Variáveis para PWM
static uint slice_num_red, slice_num_green, slice_num_blue; // Números dos slices PWM
static uint chan_red, chan_green, chan_blue; // Canais PWM

// Inicializa o PWM para os LEDs
void init_pwm() {
    gpio_set_function(LED_RED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_GREEN_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LED_BLUE_PIN, GPIO_FUNC_PWM);
    slice_num_red = pwm_gpio_to_slice_num(LED_RED_PIN);
    slice_num_green = pwm_gpio_to_slice_num(LED_GREEN_PIN);
    slice_num_blue = pwm_gpio_to_slice_num(LED_BLUE_PIN);
    chan_red = pwm_gpio_to_channel(LED_RED_PIN);
    chan_green = pwm_gpio_to_channel(LED_GREEN_PIN);
    chan_blue = pwm_gpio_to_channel(LED_BLUE_PIN);
    pwm_set_wrap(slice_num_red, 65535);
    pwm_set_wrap(slice_num_green, 65535);
    pwm_set_wrap(slice_num_blue, 65535);
    pwm_set_enabled(slice_num_red, true);
    pwm_set_enabled(slice_num_green, true);
    pwm_set_enabled(slice_num_blue, true);
}

// Atualiza a intensidade dos LEDs com base na posição do joystick
void update_pwm(uint16_t x_value, uint16_t y_value) {
    if (!pwm_enabled) return;

    int16_t x_centered = x_value - 2048; // Centraliza o valor do eixo X
    uint16_t red_intensity = (abs(x_centered) > DEADZONE_X) ?
        (uint16_t)(((uint32_t)abs(x_centered) * 65535) / (2048 - DEADZONE_X)) : 0;
    if (red_intensity > 65535) red_intensity = 65535;
    pwm_set_chan_level(slice_num_red, chan_red, red_intensity);

    int16_t y_centered = y_value - 2048; // Centraliza o valor do eixo Y
    uint16_t blue_intensity = (abs(y_centered) > DEADZONE_Y) ?
        (uint16_t)(((uint32_t)abs(y_centered) * 65535) / (2048 - DEADZONE_Y)) : 0;
    if (blue_intensity > 65535) blue_intensity = 65535;
    pwm_set_chan_level(slice_num_blue, chan_blue, blue_intensity);

    if (local_ok && !target_acquired) {
        uint16_t green_intensity = (uint16_t)((float)altura * (65535.0f / 100.0f));
        if (green_intensity > 65535) green_intensity = 65535;
        pwm_set_chan_level(slice_num_green, chan_green, green_intensity);
    } else {
        pwm_set_chan_level(slice_num_green, chan_green, 0);
    }
}

// Função de debounce para evitar leituras múltiplas de um único pressionamento
bool debounce(uint gpio, uint32_t *last_time) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - *last_time < 200) {
        return false;
    }
    *last_time = current_time;
    return !gpio_get(gpio);
}

// Função de callback para interrupções dos botões
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == JOYSTICK_PB && debounce(gpio, &last_time_joystick)) {
        if (!altura_ok) {
            if (altura == alvo_altura) {
                altura_ok = true;
                local_ok = true;
                exibir_alvo = true;
                exibir_alvo_start_time = to_ms_since_boot(get_absolute_time());
                printf("Altura confirmada! Modo de localização ativado.\n");
            } else {
                printf("Altura nao atingida! (Atual: %d, Alvo: %d)\n", altura, alvo_altura);
            }
        }
        else if (in_position && local_ok) {
            capture_mode = !capture_mode;
            if (capture_mode) {
                pwm_enabled = false;
                printf("Modo de captura ativado!\n");
            } else {
                pwm_enabled = true;
                printf("Modo de captura desativado!\n");
            }
        }
        else if (target_acquired) {
            target_acquired = false;
            pwm_enabled = true;
            altura = 0;
            alvo_x = rand() % DISPLAY_WIDTH;
            alvo_y = rand() % DISPLAY_HEIGHT;
            alvo_altura = rand() % 101;
            capture_mode = false;
            in_position = false;
            altura_ok = false;
            local_ok = false;
            cursor_x = DISPLAY_WIDTH / 2;
            cursor_y = DISPLAY_HEIGHT / 2;
            printf("Novo alvo: (%d, %d, %d)\n", alvo_x, alvo_y, alvo_altura);
        }
    }

    if (!altura_ok) {
        if (gpio == BUTTON_A_PIN && debounce(BUTTON_A_PIN, &last_time_buttonA)) {
            if (altura > 0) altura--;
            printf("Botão A: Altura: %d\n", altura);
        }
        if (gpio == BUTTON_B_PIN && debounce(BUTTON_B_PIN, &last_time_buttonB)) {
            if (altura < 100) altura++;
            printf("Botão B: Altura: %d\n", altura);
        }
    }
}

// Função para desenhar um caractere no display OLED
void desenhar_caractere(ssd1306_t *ssd, char c, uint8_t x, uint8_t y) {
    int font_index = -1;
    if (c >= 'A' && c <= 'Z')
        font_index = (c - 'A' + 11) * 8;
    else if (c >= '0' && c <= '9')
        font_index = (c - '0' + 1) * 8;
    else if (c >= 'a' && c <= 'z')
        font_index = (c - 'a' + 37) * 8;
    if (font_index != -1) {
        for (uint8_t i = 0; i < 8; ++i) {
            uint8_t line = font[font_index + i];
            for (uint8_t j = 0; j < 8; ++j) {
                ssd1306_pixel(ssd, x + i, y + j, line & (1 << j));
            }
        }
    }
}

// Função para desenhar uma string no display OLED
void desenhar_string(ssd1306_t *ssd, const char *str, uint8_t x, uint8_t y) {
    while (*str) {
        desenhar_caractere(ssd, *str++, x, y);
        x += 8;
        if (x + 8 >= ssd->width) { x = 0; y += 8; }
        if (y + 8 >= ssd->height) break;
    }
}

// Função para desenhar uma string centralizada no display OLED
void desenhar_string_centralizado(ssd1306_t *ssd, const char *str, uint8_t y) {
    int str_len = 0;
    const char *temp = str;
    while (*temp++) { str_len++; }
    int x = (ssd->width - (str_len * 8)) / 2;
    desenhar_string(ssd, str, x, y);
}

// Função para desenhar a borda do display
void draw_border(void) {
    switch (border_style) {
        case 0: ssd1306_rect(&display, 0, 0, 128, 64, true, false); break;
        case 1: ssd1306_rect(&display, 2, 2, 124, 60, true, false); break;
        case 2: ssd1306_rect(&display, 4, 4, 120, 56, true, false); break;
    }
}

// Função para desenhar um quadrado no display OLED
void draw_square(ssd1306_t *ssd, int x, int y, int size, bool color) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            int xx = x + i;
            int yy = y + j;
            if (xx >= 0 && xx < ssd->width && yy >= 0 && yy < ssd->height) {
                ssd1306_pixel(ssd, xx, yy, color);
            }
        }
    }
}

// Função para desenhar um pixel no display OLED
void draw_pixel(ssd1306_t *ssd, int x, int y, bool color) {
    if (x >= 0 && x < ssd->width && y >= 0 && y < ssd->height) {
        ssd1306_pixel(ssd, x, y, color);
    }
}

// Fator de escala para reduzir a velocidade do movimento do cursor
#define SLOW_FACTOR 0.05f

// Função principal
int main(void) {
    stdio_init_all(); // Inicializa a comunicação serial (USB)

    // Inicializações de hardware
    adc_init(); // Inicializa o ADC
    adc_gpio_init(JOYSTICK_X_PIN); // Configura o pino do eixo X do joystick
    adc_gpio_init(JOYSTICK_Y_PIN); // Configura o pino do eixo Y do joystick
    init_pwm(); // Inicializa o PWM para os LEDs
    i2c_init(I2C_PORT, 400 * 1000); // Inicializa o I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Configura o pino SDA
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Configura o pino SCL
    gpio_pull_up(I2C_SDA); // Habilita pull-up no pino SDA
    gpio_pull_up(I2C_SCL); // Habilita pull-up no pino SCL
    ssd1306_init(&display, DISPLAY_WIDTH, DISPLAY_HEIGHT, false, SSD1306_ADDRESS, I2C_PORT); // Inicializa o display OLED
    ssd1306_config(&display); // Configura o display
    ssd1306_fill(&display, false); // Limpa o display
    ssd1306_send_data(&display); // Envia os dados para o display

    // Inicialização dos botões e interrupções
    gpio_init(JOYSTICK_PB);
    gpio_set_dir(JOYSTICK_PB, GPIO_IN);
    gpio_pull_up(JOYSTICK_PB);
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);
    gpio_set_irq_enabled_with_callback(JOYSTICK_PB, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Define valores iniciais aleatórios para o alvo
    srand(to_us_since_boot(get_absolute_time()));
    alvo_x = rand() % DISPLAY_WIDTH;
    alvo_y = rand() % DISPLAY_HEIGHT;
    alvo_altura = rand() % 101;
    printf("Alvo inicial: (%d, %d, %d)\n", alvo_x, alvo_y, alvo_altura);

    // Inicializa o cursor no centro do display
    cursor_x = (DISPLAY_WIDTH - SQUARE_SIZE) / 2;
    cursor_y = (DISPLAY_HEIGHT - SQUARE_SIZE) / 2;

    uint32_t last_move_time = 0;
    const uint32_t move_interval = 50; // Intervalo de movimento (não utilizado com mapeamento direto)

    // Loop principal
    while (true) {
        if (exibir_alvo) {
            // Exibe as coordenadas do alvo no display
            ssd1306_fill(&display, false);
            char alvo_coords_str[30];
            sprintf(alvo_coords_str, "Alvo: (%d, %d)", alvo_x, alvo_y);
            desenhar_string_centralizado(&display, alvo_coords_str, 20);
            draw_border();
            ssd1306_send_data(&display);
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (current_time - exibir_alvo_start_time >= TEMPO_EXIBICAO_ALVO) {
                exibir_alvo = false;
            }
        }
        else if (!altura_ok) {
            // Exibe a altura atual e a altura do alvo no display
            ssd1306_fill(&display, false);
            char altura_str[20];
            sprintf(altura_str, "Altura: %d", altura);
            desenhar_string_centralizado(&display, altura_str, 20);
            char alvo_altura_str[20];
            sprintf(alvo_altura_str, "Alvo Alt: %d", alvo_altura);
            desenhar_string_centralizado(&display, alvo_altura_str, 10);
            draw_border();
            ssd1306_send_data(&display);
        }
        else if (local_ok) {
            // Leitura dos valores ADC (joystick)
            adc_select_input(0);
            adc_value_x = adc_read();
            adc_select_input(1);
            adc_value_y = adc_read();

            // Calcula os valores centralizados (0 quando no centro)
            int centered_x = adc_value_x - 2048;
            int centered_y = adc_value_y - 2048;

            // Aplica a rotação de -90º e inversão dos eixos
            int rotated_x = -centered_y;
            int rotated_y = centered_x;

            // Aplica inversão multiplicando por -1 no cálculo da posição
            cursor_x = (DISPLAY_WIDTH - SQUARE_SIZE) / 2 - (int)(rotated_x * SLOW_FACTOR);
            cursor_y = (DISPLAY_HEIGHT - SQUARE_SIZE) / 2 - (int)(rotated_y * SLOW_FACTOR);

            // Garante que o cursor permaneça dentro dos limites do display
            if (cursor_x < 0) cursor_x = 0;
            if (cursor_x > DISPLAY_WIDTH - SQUARE_SIZE) cursor_x = DISPLAY_WIDTH - SQUARE_SIZE;
            if (cursor_y < 0) cursor_y = 0;
            if (cursor_y > DISPLAY_HEIGHT - SQUARE_SIZE) cursor_y = DISPLAY_HEIGHT - SQUARE_SIZE;

            update_pwm(adc_value_x, adc_value_y); // Atualiza a intensidade dos LEDs

            // Verifica se o alvo está dentro da área do cursor
            if (alvo_x >= cursor_x && alvo_x <= cursor_x + SQUARE_SIZE &&
                alvo_y >= cursor_y && alvo_y <= cursor_y + SQUARE_SIZE)
            {
                in_position = true;
            } else {
                in_position = false;
            }

            if (capture_mode && !target_acquired) {
                // Verifica se o alvo foi capturado
                if (alvo_x >= cursor_x && alvo_x <= cursor_x + SQUARE_SIZE &&
                    alvo_y >= cursor_y && alvo_y <= cursor_y + SQUARE_SIZE)
                {
                    target_acquired = true;
                    capture_mode = false;
                    printf("Alvo Adquirido!\n");
                }
            }

            ssd1306_fill(&display, false);
            if (!target_acquired) {
                if (in_position) {
                    desenhar_string_centralizado(&display, "MODO CAPTURA", 30);
                }
                draw_pixel(&display, alvo_x, alvo_y, true); // Desenha o alvo
                draw_square(&display, cursor_x, cursor_y, SQUARE_SIZE, true); // Desenha o cursor
            } else {
                desenhar_string_centralizado(&display, "CAPTURADO!", 15);
            }
            draw_border();
            ssd1306_send_data(&display);
        }
        else if (target_acquired) {
            // Aguarda 5 segundos e gera um novo alvo
            sleep_ms(5000);
            target_acquired = false;
            pwm_enabled = true;
            altura = 0;
            alvo_x = rand() % DISPLAY_WIDTH;
            alvo_y = rand() % DISPLAY_HEIGHT;
            alvo_altura = rand() % 101;
            capture_mode = false;
            in_position = false;
            altura_ok = false;
            local_ok = false;
            cursor_x = (DISPLAY_WIDTH - SQUARE_SIZE) / 2;
            cursor_y = (DISPLAY_HEIGHT - SQUARE_SIZE) / 2;
            printf("Novo alvo: (%d, %d, %d)\n", alvo_x, alvo_y, alvo_altura);
        }
        sleep_ms(10); // Pequena pausa para evitar uso excessivo da CPU
    }
    return 0;
}