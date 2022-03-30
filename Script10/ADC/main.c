//================================================================================ //
// Sprint 9 de  Yuri Dantas
//================================================================================ //

#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/delay.h>
#include "nokia5110.h"
#include "Font5x8.h"
#include "SSD1306.h"
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


//variaveis globais

uint8_t flag_5ms=0,flag_500ms=0,adc_flag=1;
uint16_t RPM_motor=0, Velocidade_carro_KmH=100,Diametro_pneu_cm = 60, Modo_operacao=0, Modo_operacao1=0;
uint32_t leitura_ADC2=0,tempo_ms = 0,tempo_novo=0, tempo_borda_subida, tempo_delta,distancia_sonar=0;
float Distancia_hodometro_km = 0,kilometragemAtual=0, apoio = 0, apoio1 = 0;
uint16_t leitura_ADC = 0,leitura_ADC1 = 0, limpaeeprom = 0, tela = 0, farol = 0;


//prototipo
void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo);
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo);
void EEPROM_write(unsigned int endereco, unsigned char Dadosalvo); //prototipo para usar as funções de escrita
unsigned char EEPROM_read(unsigned int endereco); // prototipo para usar as funções de leitura do eeprom

//Tratamento de interrupções

ISR(PCINT1_vect){ // interrupção de C para resetar a kilometragem atual e modulos do farol 'video
	if((PINC&0b01000000)==0){ //C6
		kilometragemAtual = 0;
		eeprom_write_byte(3,kilometragemAtual);
	}
	if((PINC&0b00001000)==0){ //C3
		farol = (farol+1)%4;
		eeprom_write_byte(4,farol);
	}
	
}

ISR(USART_RX_vect)
{
	char recebido;
	recebido = UDR0; //UDR0 contém o dado recebido via USART
	if(recebido == 'd'){
	eeprom_write_byte(2,leitura_ADC2);
	USART_Transmit(eeprom_read_byte(2));
	USART_Transmit("Y");
	}
	else if (recebido == 'l')
	{
		eeprom_write_byte(2,limpaeeprom);
	}
}

ISR(TIMER0_COMPA_vect) //interrupção do TC0 a cada 1ms = (64*(249+1))/16MHz
{
	tempo_ms++;
	//PORTD ^= 0b01000000;
	if ((tempo_ms % 5)== 0)
		flag_5ms = 1;
	if((tempo_ms % 500)==0)
		flag_500ms = 1;
}

ISR(INT0_vect) //interrupção externa 0, PIN D2
{
	static uint8_t cont_5voltas = 0;
	static uint32_t tempo_ms_anterior = 0;
	uint16_t delta_t_ms = 0;

	if(cont_5voltas ==5)
	{
		delta_t_ms = tempo_ms - tempo_ms_anterior;
		RPM_motor = 300000/(delta_t_ms); //(5voltas*60min*1000)/delta_t_ms
		Velocidade_carro_KmH = ((uint32_t)Diametro_pneu_cm*565)/delta_t_ms; //*3.1415*3.6/(60*100)
		tempo_ms_anterior = tempo_ms;
		cont_5voltas = 0;
	}	
	cont_5voltas++;
	Distancia_hodometro_km += ((float)Diametro_pneu_cm*3.1415)/100000; //distancia percorrida por 1 volta do pneu, em km
	kilometragemAtual += ((float)Diametro_pneu_cm*3.1415)/100000; //kimoletragem atual 'video
	eeprom_write_byte(0,Distancia_hodometro_km); //Salvando kilometragem depois da alteração no endereço 0
	eeprom_write_byte(3,kilometragemAtual); //Salvando a kilometragem Atual no endereço 3 'video
}

ISR(PCINT2_vect) //interrupção por mudança de pino na porta D (PD4 e PD5 - Botão "+" e "-" e PD6 PD7 - Cambio do carro)
{
	if((PIND&0b00010000)==0) //Botão "+" pressionado (PD4)
	{
		if (Diametro_pneu_cm < 200) //maximo de 2m
			Diametro_pneu_cm++;	
			eeprom_write_byte(1,Diametro_pneu_cm); //Salvando o Diametro do pneu depois do Aumento no endereço 1
	}
	if ((PIND&0b00100000)==0)  //Botão "-" pressionado (PD5)
	{
		if(Diametro_pneu_cm > 1)
			Diametro_pneu_cm--;
			eeprom_write_byte(1,Diametro_pneu_cm); //Salvando o Diametro do pneu depois da diminuição no endereço 1
	}
	if ((PIND&0b00110000)==0) // caso ambos as portas sofrerem interrupção entra no if 'video 
	{
		tela = (tela +1)%2;
	}
	
	if ((PIND&0b01000000)==0)            //Cambio do carro -> d e p 0 e 1
	{
		Modo_operacao=0; // 0 = D
	}
	else
	Modo_operacao=1; // 1 = R
	
	if ((PIND&0b10000000)==0)
	{
		Modo_operacao1 = 1; // 1 = P
	}
	else 
	   Modo_operacao1 = 0;
}

ISR(TIMER1_CAPT_vect) //interrupção do timer para fazer a distancia no sonar 
{
	if(TCCR1B & (1<<ICES1))
	tempo_borda_subida =ICR1;
	else
	tempo_delta = (ICR1 - tempo_borda_subida)*4;
	distancia_sonar = tempo_delta/14.53; //regra de 3 para que o valor de distancia_sonar seja igual a fonte de tensão 
	TCCR1B ^=(1<<ICES1);
} 

ISR(ADC_vect)
{
	if(tempo_ms-tempo_novo>50) // tempo suficiente para que a conversão seja realizada
	tempo_novo=tempo_ms;
	if (adc_flag==1)
	{
		ADMUX = 0b01000000; //mudança do admux para utilização do C0
		if(distancia_sonar>300){ // if para limitar o o giro do motor
			leitura_ADC = ADC;
			OCR2B = leitura_ADC/4;
			}else if (distancia_sonar<300){
			leitura_ADC = ADC;
			OCR2B = 26;//aproximadamente 10% de 256
		}
		if(tempo_novo==tempo_novo)
		adc_flag =2;
	}
	else if (adc_flag==2)
	{
		ADMUX = 0b01000001; //mudança do admux para utilização do C1
		leitura_ADC1 = (ADC*25)/256;
		if(tempo_novo==tempo_novo)
		adc_flag =3;
	}
	else if (adc_flag==3)
	{
		ADMUX = 0b01000010; //mudança do admux para utilização do C2
		apoio = 0.00475*ADC+0.0179; //equação De Adc para V
		apoio1 = (1000*apoio)/(5-apoio); //equação de V para Rt
		leitura_ADC2 = 2.66*apoio1-269.65; //equação de Rt para T
		if(tempo_novo==tempo_novo)
		adc_flag =1;
	}
	
	
}


int main(void)
{
	USART_Init(MYUBRR);

	OCR2B = 128; //controle do ciclo ativo do PWM OCR2B (PD5)
	
	//Definições de GPID
		//Direção dos pinos
	DDRB |= 0b11111110; //Habilita os pinos PB0..6 como saídas
	DDRD = 0xFF;
	DDRD &= 0b00001011; //Habilita PD2 ... PD7 como entradas
	DDRC &= 0b11111110; // Habilita PD0
	
		//Pullups das entradas
	PORTD = 0b00111100; //Habilita pullups de PD2..5
	PORTC = 0xFE; //Desabilita o pullup do PC0
	
	//configuração das interrupções externas
	EICRA = 0b00000010; //interrupção externa INT0 na borda de descida
	EIMSK = 0b00000001; //Habilita a interrupção externa INT0
	PCICR = 0b00000110; //habilita interrup pin change 2 (porta D e porta C) 'video
	PCMSK2= 0b11111100; //habilita interrup PnChange PD
	PCMSK1= 0b01001000; //Habilita interrup PnChange PC6 e PC3
	
	//configuração do Timer 0
	TCCR0A = 0b00000010; //habilita modo CTC do TC0
	TCCR0B = 0b00000011; //liga TC0 com prescaler = 256
	OCR0A = 249;		 //ajusta o comparador para TC0 contar até 249
	TIMSK0 = 0b00000010; //habilita a interrupção na igualdade de comparação com OCR0A. A interrupção ocorre a cada 1ms
	TIMSK1 = 1<<ICIE1;
	//Configuração do PWM
	TCCR2A = 0b00100011; //habilita modo CTC do TC2
	TCCR2B = 0b00000100; //liga TC2 com prescaler = 256
	TCCR1B = (1<<ICES1)| (1<<CS12);
	
	//Configura ADC
	ADMUX = 0b01000000; //Tensão interna de ref (1.1V), canal 0
	ADCSRA = 0b11101111; //habilita o AD, habilita interrupção, modo de conversão contínua, prescaler = 128
	ADCSRB = 0x00; //modo de conversão contínua
	DIDR0 = 0b00000001; //habilita pino PC0 como entrada do ADC0
	
	//Atualizando as variaveis atraves do eeprom
	Distancia_hodometro_km = eeprom_read_byte(0);
	Diametro_pneu_cm = eeprom_read_byte(1);
	kilometragemAtual = eeprom_read_byte(3);
	farol = eeprom_read_byte(4);
	
	
	UCSR0B |= (1<<RXCIE0);//Habilita a interrupção de recepção da USART
	//Habilita o flag de interrupções globais
	sei();
	//inicialização do LCD
			GLCD_Setup();
			GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
			GLCD_InvertScreen(); //de preto pra branco 
	
	while(1)
	{
		anima_velocidade(Velocidade_carro_KmH, &flag_5ms);
		anima_LCD(Diametro_pneu_cm, RPM_motor, Distancia_hodometro_km, &flag_500ms);
	}
}

// Inicializando o USART
void USART_Init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //Habilita o transmissor e o receptor
	UCSR0C = (0 << USBS0) | (3 << UCSZ00) << (0 << UPM01) << (0 << UPM00); //8bits, 1 bit de parada, sem paridade
}

// Função de envio do USART
void USART_Transmit(unsigned char data)
{
	while(!( UCSR0A & (1<<UDRE0)));//Espera a limpeza do registr. de transmissão
	UDR0 = data; //Coloca o dado no registrador e o envia
}

// Função de recebimento do USART
unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0))); //Espera o dado ser recebido
	return UDR0; //Lê o dado recebido e retorna
}

void anima_velocidade(uint16_t velocidade_carro, uint8_t *flag_disparo)
{
	static int8_t cont_dig = 0;
	if(*flag_disparo)
	{
		switch (cont_dig)
		{
			case 0:
				PORTB &= 0b00000001;
				PORTB |= 0b11000000;
				PORTB |= (((velocidade_carro/1)%10) & 0b00011110);
				break;
			case 1:
				PORTB &= 0b00000001;
				PORTB |= 0b10100000;
				PORTB |= (((velocidade_carro/10)%10) & 0b00011110);
				break;
			case 2:
				PORTB &= 0b00000001;
				PORTB |= 0b01100000;
				PORTB |= (((velocidade_carro/100)%10) & 0b00011110);
				cont_dig = -1;
				break;
		}
		cont_dig++;
		*flag_disparo = 0;
	}
}
void anima_LCD(uint16_t diametro_pneu_cm, uint16_t rpm_motor, float distancia_hodometro_km, uint8_t *flag_disparo)
{
	if(*flag_disparo)
	{
		
		unsigned char diametro_pneu_cm_string[4];
		unsigned char leitura_ADC_string[4];
		unsigned char leitura_ADC1_string[4];
		unsigned char leitura_ADC2_string[4];
		unsigned char rpm_motor_string[6];
		unsigned char distancia_sonar_string[4];
		unsigned char distancia_hodometro_km_string[8];
		unsigned char kilometragemAtual_string[5];
		
		
		sprintf(diametro_pneu_cm_string, "%u", Diametro_pneu_cm);
		sprintf(distancia_sonar_string, "%u", distancia_sonar);
		sprintf(rpm_motor_string, "%u", RPM_motor);
		sprintf(leitura_ADC_string, "%u", leitura_ADC);
		sprintf(leitura_ADC1_string, "%u", leitura_ADC1);
		sprintf(leitura_ADC2_string, "%u", leitura_ADC2);
		sprintf(distancia_hodometro_km_string, "%u", (uint16_t)distancia_hodometro_km);
		sprintf(kilometragemAtual_string, "%u", (uint16_t)kilometragemAtual);
		
		// Design do visor GLCD
		GLCD_Clear();
		if (tela == 0)
		{
		
		GLCD_GotoXY(0,0);
		GLCD_PrintString("LASD CAR");
		GLCD_DrawLine(0, 8, 50, 8, GLCD_Black);
		GLCD_GotoXY(0,20);
		GLCD_PrintString(rpm_motor_string);
		GLCD_GotoXY(30,20);
		GLCD_PrintString("RPM");
		GLCD_GotoXY(0,35);
		GLCD_PrintString("Sonar:");
		GLCD_GotoXY(35,35);
		GLCD_PrintString(distancia_sonar_string);
		GLCD_GotoXY(0,50);
		GLCD_PrintString("D.pneu:");
		GLCD_GotoXY(40,50);
		GLCD_PrintString(diametro_pneu_cm_string);
		GLCD_GotoXY(85,40);
		GLCD_PrintString(distancia_hodometro_km_string);
		GLCD_GotoXY(105,40);
		GLCD_PrintString("KM");
		GLCD_DrawRectangle(80,0,122,30, GLCD_Black);
		GLCD_GotoXY(85,5);
		GLCD_PrintString(leitura_ADC1_string);
		GLCD_GotoXY(100,5);
		GLCD_PrintString("%");
		GLCD_GotoXY(85,20);
		GLCD_PrintString(leitura_ADC2_string);
		GLCD_GotoXY(105,20);
		GLCD_PrintString("C");
		
		if (Modo_operacao1 ==0)
		{
			if (Modo_operacao==0)
			{
				GLCD_GotoXY(120,50);
				GLCD_PrintString("D");
			}
			else if (Modo_operacao ==1)
			{ 
			GLCD_GotoXY(120,50);
			GLCD_PrintString("R");
			}
		}
		else{
			GLCD_GotoXY(120,50);
			GLCD_PrintString("P");
		}
		
		
		
		GLCD_Render();
		*flag_disparo = 0;
		}else if (tela == 1)
		{
			GLCD_GotoXY(0,0);
			GLCD_PrintString("LASD CAR");
			GLCD_DrawLine(0, 8, 50, 8, GLCD_Black);
			GLCD_GotoXY(85,0);
			GLCD_PrintString(leitura_ADC1_string);
			GLCD_GotoXY(100,0);
			GLCD_PrintString("%");
			GLCD_GotoXY(85,10);
			GLCD_PrintString(leitura_ADC2_string);
			GLCD_GotoXY(105,10);
			GLCD_PrintString("C");
			GLCD_GotoXY(0,15);
			GLCD_PrintString("Percurso:");
			GLCD_GotoXY(54,15);
			GLCD_PrintString(kilometragemAtual_string);
			GLCD_GotoXY(70,15);
			GLCD_PrintString("Km");
			switch (farol){
				case 0:
					GLCD_GotoXY(20,30);
					GLCD_PrintString("Farol");
					GLCD_GotoXY(10,40);
					GLCD_PrintString("Desligado");
					break;
				case 1:
					GLCD_DrawRoundRectangle(10,30,30,50,8,GLCD_Black);
					GLCD_DrawLine(25,30,20,50,GLCD_Black);
					GLCD_DrawLine(28,35,40,40,GLCD_Black);
					//GLCD_DrawLine(28,38,40,43,GLCD_Black);
					GLCD_DrawLine(28,41,40,46,GLCD_Black);
					//GLCD_DrawLine(28,44,40,49,GLCD_Black);
					GLCD_DrawLine(28,47,40,52,GLCD_Black);
					GLCD_DrawLine(34,30,34,55,GLCD_Black); //corta neblina
					GLCD_DrawLine(25,30,35,38,GLCD_White);
					GLCD_DrawLine(25,50,35,38,GLCD_White);
					GLCD_DrawLine(21,50,28,50,GLCD_White);
					GLCD_DrawLine(28,45,32,50,GLCD_White);
					GLCD_DrawLine(30,30,30,50,GLCD_White);
					GLCD_GotoXY(2,56);
					GLCD_PrintString("Neblina");
					break;
				case 2:
						GLCD_DrawRoundRectangle(10,30,30,50,8,GLCD_Black);
						GLCD_DrawLine(25,30,20,50,GLCD_Black);
						GLCD_DrawLine(28,35,40,40,GLCD_Black);
						GLCD_DrawLine(28,38,40,43,GLCD_Black);
						GLCD_DrawLine(28,41,40,46,GLCD_Black);
						GLCD_DrawLine(28,44,40,49,GLCD_Black);
						GLCD_DrawLine(28,47,40,52,GLCD_Black);
						GLCD_DrawLine(25,30,35,38,GLCD_White);
						GLCD_DrawLine(25,50,35,38,GLCD_White);
						GLCD_DrawLine(21,50,28,50,GLCD_White);
						GLCD_DrawLine(28,45,32,50,GLCD_White);
						GLCD_DrawLine(30,30,30,50,GLCD_White);
						GLCD_GotoXY(2,56);
						GLCD_PrintString("Baixo");
					break;
				case 3:
						GLCD_DrawRoundRectangle(10,30,30,50,8,GLCD_Black);
						GLCD_DrawLine(25,30,20,50,GLCD_Black);
						GLCD_DrawLine(28,35,40,35,GLCD_Black);
						GLCD_DrawLine(28,38,40,38,GLCD_Black);
						GLCD_DrawLine(28,41,40,41,GLCD_Black);
						GLCD_DrawLine(28,44,40,44,GLCD_Black);
						GLCD_DrawLine(28,47,40,47,GLCD_Black);
						GLCD_DrawLine(25,30,35,38,GLCD_White);
						GLCD_DrawLine(25,50,35,38,GLCD_White);
						GLCD_DrawLine(21,50,28,50,GLCD_White);
						GLCD_DrawLine(28,45,32,50,GLCD_White);
						GLCD_DrawLine(30,30,30,50,GLCD_White);
						GLCD_GotoXY(2,56);
						GLCD_PrintString("Alto");
					break;
			}
			
			if (leitura_ADC2>= 100)
			{
			GLCD_DrawTriangle(80,45,100,20,120,45,GLCD_Black);
			GLCD_DrawRectangle(98,38,102,24,GLCD_Black);
			GLCD_FillRectangle(98,24,102,38,GLCD_Black);
			GLCD_DrawRectangle(98,41,102,44,GLCD_Black);
			GLCD_FillRectangle(98,41,102,44,GLCD_Black);
			GLCD_GotoXY(75,48);
			GLCD_PrintString("Bateria");
			GLCD_GotoXY(50,56);
			GLCD_PrintString("Superaquecida");
	
			}
						GLCD_Render();
			*flag_disparo = 0;
		}
	}
}
