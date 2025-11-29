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

/*
    TO-DO LIST GIGANTE DE MDF SUPER MANEIRO IRADO

    Laser:
    - [OK] Toggle do laser usando timer (1 Hz)

    LDR e Dano:
    - [OK] Leitura contínua do LDR
    - [OK] Detecção de dano (threshold)
    - [OK] Ao tomar dano: parar o carrinho
    - [OK] Girar 180°
    - [OK] Aguardar 5 segundos antes de voltar ao normal

    Sistema de Vidas:
    - [OK] Variável de vidas
    - [OK] RGB: verde (3 vidas), amarelo (2), vermelho (1), desligado (0)
    - [OK] Reduzir vida ao tomar dano
    - [OK] Travar carrinho ao sofrer dano

    Controle Bluetooth:
    - [OK] UART por software (RX/TX digitais)
    - [OK] Função de leitura UART
    - [OK] Mapeamento dos comandos (frente, trás, esquerda, direita, parar)
    - [OK] Integração dos comandos BT com os motores

    Movimentação:
    - [OK] Controle dos motores
    - [OK] Função girar 180°
    - [OK] Garantir giro independente da direção atual

    Finalização:
    - [OK] Teste final
*/


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
#define MAX_LIFES 3
#define LDR_THRESHOLD 700

int CurLifes = MAX_LIFES;
int LaserOn = 0;
int takenDamage = 0;

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
	if (LaserOn && CurLifes>0 && !takenDamage) {
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
	
	//motor1 PWM + IN1 + IN2
	DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB4) ;

	// motor2 PWM motor2 IN1 + IN2
	DDRB |= (1 << PB5);
	DDRD |= (1 << PD4) | (1 << PD5);
	
}

void frente() {
	SerialPrintln("indo pra frente");
	// Motor 1
	PORTB &= ~(1 << MOTOR1_IN1);   
	PORTB |=  (1 << MOTOR1_IN2);   
	PORTB |=  (1 << MOTOR1_PWM);   

	// Motor 2 (invertido)
	PORTD &= ~(1 << MOTOR2_IN1);
	PORTD |=  (1 << MOTOR2_IN2); 
	PORTB |=  (1 << MOTOR2_PWM);
}

void tras() {
	SerialPrintln("indo pra tras");
	//Motor1
	PORTB |=  (1 << MOTOR1_IN1);  
	PORTB &= ~(1 << MOTOR1_IN2); 
	PORTB |=  (1 << MOTOR1_PWM);

	//Motor 2 (invertido)
	PORTD |=  (1 << MOTOR2_IN1);  
	PORTD &= ~(1 << MOTOR2_IN2); 
	PORTB |=  (1 << MOTOR2_PWM);
}

void direita() {
	SerialPrintln("indo pra direit");
	// Motor 1 (esquerdo) ligado pra frente
	PORTB &= ~(1 << MOTOR1_IN1);
	PORTB |=  (1 << MOTOR1_IN2);
	PORTB |=  (1 << MOTOR1_PWM);

	// Motor 2 (direito) desligado
	PORTD &= ~(1 << MOTOR2_IN1);
	PORTD &= ~(1 << MOTOR2_IN2);
	PORTB &= ~(1 << MOTOR2_PWM);
}

void esquerda() {
	SerialPrintln("indo pra esquerda");
	// Motor 1 (esquerdo) desligado
	PORTB &= ~(1 << MOTOR1_IN1);
	PORTB &= ~(1 << MOTOR1_IN2);
	PORTB &= ~(1 << MOTOR1_PWM);

	// Motor 2 (direito) ligado pra frente (invertido)
	PORTD &= ~(1 << MOTOR2_IN1);
	PORTD |=  (1 << MOTOR2_IN2);
	PORTB |=  (1 << MOTOR2_PWM);
}

void Virar180() {
	SerialPrintln("virando 180");
	// MOTOR 1 ? frente
	PORTB &= ~(1 << MOTOR1_IN1);   // LOW
	PORTB |=  (1 << MOTOR1_IN2);   // HIGH
	PORTB |=  (1 << MOTOR1_PWM);   // LIGA

	// MOTOR 2 ? trás (invertido)
	PORTD |=  (1 << MOTOR2_IN1);   // HIGH
	PORTD &= ~(1 << MOTOR2_IN2);   // LOW
	PORTB |=  (1 << MOTOR2_PWM);   // LIGA
	
	_delay_ms(1100);
	PararMotores();
}

void PararMotores() {
	SerialPrintln("parando");
	// Motor 1
	PORTB &= ~(1 << MOTOR1_IN1);
	PORTB &= ~(1 << MOTOR1_IN2);
	PORTB &= ~(1 << MOTOR1_PWM);

	// Motor 2
	PORTD &= ~(1 << MOTOR2_IN1);
	PORTD &= ~(1 << MOTOR2_IN2);
	PORTB &= ~(1 << MOTOR2_PWM);
}


void LifePwmSetup(){
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS00);  
	OCR0A = 0; 	
}

void ResetLifes(){
	CurLifes = MAX_LIFES;
	LedHandler();
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

void TakeDamage(){
	CurLifes--;
	LedHandler();
	takenDamage = 1;
	Virar180();
	_delay_ms(5000);
	takenDamage=0;
}

char buffer[6];
void loop() {
	//transformar int em str
	//int i = 0;
	//int temp = ldrValue;
	//do {
		//buffer[i++] = (temp % 10) + '0';
		//temp /= 10;
	//} while (temp > 0);
//
	//buffer[i] = '\0';
	//for(int j=0; j<i/2; j++){
		//char t = buffer[j];
		//buffer[j] = buffer[i-1-j];
		//buffer[i-1-j] = t;
	//}
	//SerialPrintln(buffer);
	char btChar = BT_readFiltered();
	int ldrValue = ReadLdr();
	if(ldrValue>LDR_THRESHOLD && !takenDamage && CurLifes>0){
		TakeDamage();
		return;
	}
	
	if (btChar && !takenDamage && CurLifes>0) {
		SerialPrintln("BT recebeu:");
		SerialPrintln(btChar);
		switch (btChar) {
			case 'F': frente(); break;
			case 'B': tras(); break;
			case 'R': esquerda(); break;
			case 'L': direita(); break;
			case 'S': PararMotores(); break;
			case 'Y': ResetLifes(); break;
			case 'H':
			case 'h':
			case 'J':
			case 'j':
			Virar180();
			break;
		}
		}
	//}
}


int main(void) {
	setup();
	while (1) {
		loop();
	}
}
