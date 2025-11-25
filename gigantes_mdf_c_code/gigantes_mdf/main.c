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
// [ ] Implementar UART por software (RX/TX em pinos digitais)
// [ ] Criar função de leitura UART por software
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
#define RX_HC05 9  
#define TX_HC05 9


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

void setup()
{
	
}

void loop()
{
}





//Irei definir as funções setup e loop para não precisar mexer na main e facilitar a organização.
int main(void) {
	setup();
	while (1) {
		loop();
	}
}
