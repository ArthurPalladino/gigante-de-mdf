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
// [OK] Configurar timers para o laser piscar a cada 1 segundo

// ===== LASER =====
// [OK] Implementar toggle do laser usando timer (pisca 1 Hz)

// ===== LDR E DANO =====
// [OK] Ler o LDR continuamente
// [OK] Criar detecção de "dano" (threshold)
// [ ] Ao tomar dano: parar carrinho
// [ ] Girar o carrinho 180 graus
// [ ] Aguardar 5 segundos antes de voltar ao normal

// ===== SISTEMA DE VIDAS =====
// [OK Criar variável de vidas
// [OK] LED RGB verde para 3 vidas
// [OK] LED RGB amarelo para 2 vidas
// [OK] LED RGB vermelho para 1 vida
// [OK] LED RGB desligado para 0 vidas
// [ ] Reduzir vida ao tomar dano
// [ ] Travar o carrinho sempre que tomar dano

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
// [ ] Teste final de integração geral
// ------------------------------------------------------------

//RESOLVER SITUACAO DO SOFTWARE SERIAL
#define BT_RX_PIN  PB1
#define BT_RX_DDR   DDRB
#define BT_RX_PORT  PORTB
#define BT_RX_PINR  PINB

//#define BT_TX_PIN  PB0 n precisa pq so to lendo dados do serial e nao escrevendo

//PINOS
#define MOTOR1_PWM PB2   //10
#define MOTOR2_PWM PB3   //11
#define MOTOR1_IN1 PB5   //13
#define MOTOR1_IN2 PB4   //12
#define MOTOR2_IN1 PD5   //5
#define MOTOR2_IN2 PD4   //4
#define LDR        PC1   //A1
#define LASER      PD7   //7
#define RED_PIN    PC0   //A0
#define GREEN_PIN  PD6   //6


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//Variaveis do bt
volatile uint8_t bt_receiving = 0;
volatile uint8_t bt_bitIndex = 0;
volatile uint8_t bt_currentByte = 0;
volatile uint8_t bt_hasByte = 0;
volatile uint8_t bt_syncPhase = 0;

//Variaveis de codigo
#define MAX_LIFES 3;
#define LDR_THRESHOLD 700;

int CurLifes = MAX_LIFES;
int LaserOn = 0;


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


void LedHandler() {
	switch(CurLifes) {
		case 3: 
		PORTC &= ~(1 << RED_PIN); 
		OCR0A = 255;              
		break;
		case 2:
		PORTC |= (1 << RED_PIN); 
		OCR0A = 30;               
		break;
		case 1: 
		PORTC |= (1 << RED_PIN); 
		OCR0A = 0;               
		break;
		default:
		PORTC &= ~(1 << RED_PIN); 
		OCR0A = 0;               
		break;
	}
}


void LaserHandle(){
	LaserOn = !LaserOn;
	if (LaserOn && CurLifes>0) {
		PORTD |= (1 << LASER);
	}
	else {
		PORTD &= ~(1 << LASER);
	}
}

ISR(TIMER1_COMPA_vect) {
	LaserHandle();
}

void LaserTimerSetup() {
	//
	//T = 1 sec
	//Prescaler = 1024
	//FCPU / Prescaler = 15.625 Hz
	//1 = (limite + 1) / (16.000.000 / 1024 )
	//limite = 15624
	
	//Setando prescaler 1024
	
	TCCR1A = 0;
	TCCR1B = (1 << WGM12);
	TCCR1B |= (1 << CS12) | (1 << CS10);
	OCR1A = 15624;
	TIMSK1 |= (1 << OCIE1A);
}

void SetupPins(){
	//LASER OUTPUT
	DDRD |= (1 << LASER);
	//LEDF VERMELHO
	DDRC |= (1 << RED_PIN); 
	//LED VERDE
	DDRD |= (1 << GREEN_PIN);  
}

void LifePwmSetup(){
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS00);  
	OCR0A = 0; 	
}

void ResetLifes(){
	CurLifes = MAX_LIFES;
}

void SetupAdc() {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ReadLdr() {
	ADMUX = (ADMUX & 0xF0) | 1;  
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}


void setup() {
	cli();
	SerialBegin(9600);
	SerialPrintln("Comecando");
	BT_init();
	SerialPrintln("bluetooth iniciado");
	
	
	
	SetupPins();
	LaserTimerSetup();
	LifePwmSetup();
	SetupAdc();
	LedHandler();
	sei();
}

char buffer[6];
void loop() {
	char c = BT_readFiltered();

	int ldrValue = ReadLdr();
	int i = 0;
	int temp = ldrValue;
	do {
		buffer[i++] = (temp % 10) + '0';
		temp /= 10;
	} while (temp > 0);

	buffer[i] = '\0';
	for(int j=0; j<i/2; j++){
		char t = buffer[j];
		buffer[j] = buffer[i-1-j];
		buffer[i-1-j] = t;
	}
	SerialPrintln(buffer);
	
	
	//if (c) {
		//SerialPrintln("BT recebeu:");
		//SerialPrintln(c);
//
		//if (c == 'a') {
			//SerialPrintln("igual a meu goat");
		//}
	//}
}


int main(void) {
	setup();
	while (1) {
		loop();
	}
}
