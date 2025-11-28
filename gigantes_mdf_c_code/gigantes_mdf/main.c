#define F_CPU 16000000UL

//Explicação do projeto
/*
O projeto Gigante de MDF consiste na construção de um carrinho para participar de uma batalha com laser. O sistema deve disparar um feixe de luz vermelha que pisca com período de 1 segundo, controlado por um timer.
O mecanismo de dano funciona através de um LDR, quando o sensor detectar o laser inimigo, o carrinho deve girar 180°, parar por 5 segundos e então retomar a operação. Cada acerto reduz a quantidade de vidas do carrinho.
A indicação de vidas será feita por um LED RGB, seguindo o esquema:
3 vidas -> verde
2 vidas -> amarelo
1 vida -> vermelho
0 vidas -> LED apagado
O carrinho será controlado por Bluetooth.
Esta parte representa um dos maiores desafios,
pois os pinos RX/TX do módulo Bluetooth não estão conectados aos pinos RX/TX originais do ATmega328,
já que estes estão ocupados pelo conversor USB-Serial.
Isso obriga a implementação de uma rotina própria de UART por software,
usando pinos digitais comuns e temporizadores,
sem depender das bibliotecas prontas da Arduino IDE.
*/

// ------------------------------------------------------------
// TODO LIST - Projeto Gigantes de MDF
// ------------------------------------------------------------

// ===== HARDWARE E SETUP =====
// [OK] Definir todos os pinos (motor, LDR, LED RGB, laser, BT)
// [ ] Configurar timers para o laser piscar a cada 1 segundo
// [ ] Configurar interrupções se necessário (LDR / UART software)

// ===== LASER =====
// [ ] Implementar toggle do laser usando timer (pisca 1 Hz)

// ===== LDR E DANO =====
// [ ] Ler o LDR continuamente
// [ ] Criar detecção de "dano" (threshold)
// [ ] Ao tomar dano: parar carrinho
// [ ] Girar o carrinho 180 graus
// [ ] Aguardar 5 segundos antes de voltar ao normal

// ===== SISTEMA DE VIDAS =====
// [ ] Criar variável de vidas (3 ? 0)
// [ ] LED RGB verde para 3 vidas
// [ ] LED RGB amarelo para 2 vidas
// [ ] LED RGB vermelho para 1 vida
// [ ] LED RGB desligado para 0 vidas
// [ ] Reduzir vida ao tomar dano
// [ ] Travar o carrinho ao chegar em 0 vidas

// ===== CONTROLE BLUETOOTH =====
// [OK] Implementar UART por software (RX/TX em pinos digitais)
// [OK] Criar função de leitura UART por software
// [ ] Mapear comandos do controle (frente, trás, esquerda, direita, parar)
// [ ] Integrar comandos BT ao controle do motor

// ===== MOVIMENTAÇÃO =====
// [ ] Controlar motores conforme comandos do BT
// [ ] Implementar função girar 180°
// [ ] Garantir que o giro 180° funcione independente da direção atual

// ===== ESTRUTURA GERAL =====
// [ ] Criar máquina de estados (normal, tomouDano, morto)
// [ ] Organizar o código em funções separadas
// [ ] Testar cada módulo isoladamente (laser, LDR, motores, BT)
// [ ] Teste final de integração geral

// ------------------------------------------------------------

//RESOLVER SITUACAO DO SOFTWARE SERIAL
#define BT_RX_PIN  PB1
#define BT_RX_DDR   DDRB
#define BT_RX_PORT  PORTB
#define BT_RX_PINR  PINB

//#define BT_TX_PIN  PB0 n precisa pq so to lendo dados do serial e nao escrevendo

//PINOS
#define MOTOR1_PWM 10  // PB2
#define MOTOR2_PWM 11  // PB3
#define MOTOR1_IN1 13  // PB5
#define MOTOR1_IN2 12  // PB4
#define MOTOR2_IN1 5   // PD5
#define MOTOR2_IN2 4   // PD4
#define LDR A1        // PC1
#define LASER 7       // PD7
#define RED_PIN A0        // PC0
#define GREEN_PIN 6       // PD6

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint8_t bt_receiving = 0;
volatile uint8_t bt_bitIndex = 0;
volatile uint8_t bt_currentByte = 0;
volatile uint8_t bt_hasByte = 0;
volatile uint8_t bt_syncPhase = 0;


void SerialBegin(unsigned long baud) {
	unsigned long ubrr = (F_CPU / (16UL * baud)) - 1;
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)(ubrr);
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void SerialSendChar(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

char SerialReadChar() {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

char* SerialReadString() {
	static char buffer[50];
	uint8_t i = 0;

	while (1) {
		char c = SerialReadChar();
		if (c == '\n' || c == '\r' || i >= sizeof(buffer) - 1) break;
		buffer[i++] = c;
	}

	buffer[i] = '\0';
	return buffer;
}

void SerialPrintln(const char *str) {
	while (*str) SerialSendChar(*str++);
	SerialSendChar('\r');
	SerialSendChar('\n');
}

void BT_init() {
	BT_RX_DDR &= ~(1 << BT_RX_PIN);
	BT_RX_PORT |= (1 << BT_RX_PIN);

	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT1);

	TCCR2A = (1 << WGM21);
	TCCR2B = (1 << CS21);
	OCR2A = 208;
	TIMSK2 = 0;
}

ISR(PCINT0_vect) {
	if (bt_receiving) return;

	if (!(BT_RX_PINR & (1 << BT_RX_PIN))) { 
		bt_receiving = 1;
		bt_bitIndex = 0;
		bt_currentByte = 0;
		bt_syncPhase = 1; 
		TCNT2 = 0;
		TIMSK2 |= (1 << OCIE2A);
	}
}


ISR(TIMER2_COMPA_vect) {

	if (bt_syncPhase) {
		bt_syncPhase = 0;
		TCNT2 = 0;   
		return;
	}

	uint8_t bit = (BT_RX_PINR & (1 << BT_RX_PIN)) ? 1 : 0;

	bt_currentByte >>= 1;
	if (bit) bt_currentByte |= 0x80;

	bt_bitIndex++;

	if (bt_bitIndex >= 8) {
		bt_receiving = 0;
		bt_hasByte = 1;
		TIMSK2 &= ~(1 << OCIE2A);
	}
}


uint8_t BT_available() {
	return bt_hasByte;
}

char BT_read() {
	bt_hasByte = 0;
	return bt_currentByte;
}

char BT_readFiltered() {
	if (!BT_available()) return 0;
	char c = BT_read();
	if (c == '\r' || c == '\n') {
		return 0;
	}
	return c;
}

void setup() {
	cli();
	SerialBegin(9600);
	SerialPrintln("Comecando");
	BT_init();
	SerialPrintln("bluetooth iniciado");
	sei();
}

void loop() {
	char c = BT_readFiltered();

	if (c) {
		SerialPrintln("BT recebeu:");
		SerialPrintln(c);

		if (c == 'a') {
			SerialPrintln("igual a meu goat");
		}
	}
}


int main(void) {
	setup();
	while (1) {
		loop();
	}
}
