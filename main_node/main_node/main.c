#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define f_osc 8000000
#define baud  9600
#define ubrr_content ((f_osc/(16*baud))-1)
#define tx_buffer_size 128
#define rx_buffer_size 128
#define middle_boards 1

#define FOSC 8000000 // Clock Speed gia thn seiriakh
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

char rx_buffer[rx_buffer_size];
uint8_t rx_ReadPos = 0;
uint8_t rx_WritePos = 0;

char tx_buffer[tx_buffer_size];
char string_to_send[tx_buffer_size];
char string_to_print[128];
char conv_buffer[6]; //variance is at max 1024^2/4=262144 which has 6 digits
uint8_t tx_ReadPos = 0;
uint8_t tx_WritePos = 0;

bool first;
bool dry[middle_boards];
int moist_avgs[middle_boards];
float tmp_avgs[middle_boards];
int moist_vars[middle_boards];
int tmp_vars[middle_boards];

void lcd_init_sim();
void lcd_clear();
void print(char c);

/*
char getChar(void) {
    char ret = '\0';
    if(rx_ReadPos!=rx_WritePos) {
        ret = rx_buffer[rx_ReadPos++];
        if(rx_ReadPos >= rx_buffer_size)
            rx_ReadPos=0;
    }
    return ret;
}

ISR(USART_RXC_vect){
    rx_buffer[rx_WritePos++] = UDR;

    if(rx_WritePos >= rx_buffer_size){
        rx_WritePos = 0;
    }
}

void appendSerial(char c) { //write character to buffer
    tx_buffer[tx_WritePos++] = c;
    if(tx_WritePos>=tx_buffer_size)
        tx_WritePos = 0;
}

ISR(USART_TXC_vect){ //transmit single character
    if(tx_ReadPos != tx_WritePos){
        UDR = tx_buffer[tx_ReadPos++];
    }
    if(tx_ReadPos >= tx_buffer_size){
        tx_ReadPos = 0;
    }
}

void serialWrite(char c[]) {
    for(uint8_t i=0; i<strlen(c); ++i) {
        appendSerial(c[i]); //write all characters to the buffer
    }
    if(UCSRA & (1<<UDRE)) //if buffer has been emptied reset the transmission by sending a null character
        UDR = 0;
}

void sendCommand(char command[]) {
    serialWrite(command);
    char c;
    c=getChar();
    while(c!='S'){ //wait until "success" reply from esp
        if(c=='F') { //if command execution failed re-transmit it
            for(int i=0; i<5; ++i)
                getChar(); //flush fail out of read buffer
            serialWrite(command);
        }
        c=getChar();
    }
    for(int i=0; i<8; ++i)
        getChar(); //flush success out of read buffer
}*/

void usart_init(unsigned int ubrr){
	UCSRA=0;
	UCSRB=(1<<RXEN)|(1<<TXEN);
	UBRRH=(unsigned char)(ubrr>>8);
	UBRRL=(unsigned char)ubrr;
	UCSRC=(1 << URSEL) | (3 << UCSZ0);
	return;
}

void usart_transmit(uint8_t data){
	while(!(UCSRA&(1<<UDRE)));
	UDR=data;
}

uint8_t usart_receive(){
	while(!(UCSRA&(1<<RXC)));
	return UDR;
}

void serialWrite(char c[]) {
	for(uint8_t i=0; i<strlen(c); ++i) {
		usart_transmit(c[i]); //transmit command one character at a time
		//print(c[i]); //debug
	}
}

void sendCommand(char command[]) {
	serialWrite(command);
	unsigned char c;
	
	c=usart_receive();
	//PORTB=0xFF; //debug -- never reaches this part
	//print(c); //debug
	while(c!='S'){ //wait until "success" reply from esp
		if(c=='F') { //if command execution failed re-transmit it
			for(int i=0; i<5; ++i)
			usart_receive(); //flush fail out of read buffer
			PORTB=0xFF;
			serialWrite(command);
		}
		c=usart_receive();
	}
	//PORTB=0xFF;
	for(int i=0; i<8; ++i)
	//print(c);
	usart_receive(); //flush success out of read buffer
}

void wait_ServedClient() {
	char c;
	c=usart_receive();
	while(c!='S') {
		c=usart_receive();
	}
	for(int i=0; i<12; ++i)
	usart_receive(); //flush ServedClient out of read buffer
}

ISR(TIMER1_OVF_vect) {
    char c;
    bool failed;
    int counter =0, watering_pot = 0, leds;

    if(!first) {
        for(int k=0; k<middle_boards; ++k){
            failed=false;
			counter =0;
            /*strcpy(string_to_send, "ESP:ssid:\"Middle_Board");
            itoa(k, conv_buffer, 10);
            strcat(string_to_send, conv_buffer);
            strcat(string_to_send, "\"\n");
            sendCommand(string_to_send);*/
			/*
            sprintf(string_to_send, "ESP:ssid:\"Middle_Board%d\"\n", k);
            strcpy(string_to_send, "ESP:password:\"awesomePassword\"\n");
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:sensorValue:\"Moist_avg\"[request]\n");
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:sensorValue:\"Tmp_avg\"[request]\n");
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:sensorValue:\"Moist_var\"[request]\n");
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:sensorValue:\"Tmp_var\"[request]\n");
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:connect\n"); //will wait until it can connect
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:clientTransmit\n");
            sendCommand(string_to_send);*/
            strcpy(string_to_send, "ESP:getAllValues\n");
            serialWrite(string_to_send);

            while(usart_receive() != '"') { //scan input till you find ". The number will follow
               // c = getChar();
            }
            c=usart_receive();
            if(c=='F')
                failed=true;
            while(c != '"' && !failed){
                conv_buffer[counter++]=c;
                c = usart_receive();
            }
            c = usart_receive(); // also flush '\n' out of read buffer
            if(!failed){ //fix failed case!!!!!!!!!
                for(int i=5; i>=6-counter; i--){
                    conv_buffer[i] = conv_buffer[i - (6-counter)];
                }
                for(int i=0; i<(6-counter); i++){
                    conv_buffer[i] = '0';
                }
                moist_avgs[k]=atoi(conv_buffer);
            }
            
			counter =0;
            while(usart_receive() != '"') { //scan input till you find ". The number will follow
               // c = getChar();
            }
            c=usart_receive();
            if(c=='F')
                failed=true;
            while(c != '"' && !failed){
                conv_buffer[counter++]=c;
                c = usart_receive();
            }
            c = usart_receive(); // also flush '\n' out of read buffer
            if(!failed){
                for(int i=5; i>=6-counter; i--){
                    conv_buffer[i] = conv_buffer[i - (6-counter)];
                }
                for(int i=0; i<(6-counter); i++){
                    conv_buffer[i] = '0';
                }
                tmp_avgs[k]=atof(conv_buffer);
            }
            
            counter =0;
            while(usart_receive() != '"') { //scan input till you find ". The number will follow
               // c = getChar();
            }
            c=usart_receive();
            if(c=='F')
                failed=true;
            while(c != '"' && !failed){
                conv_buffer[counter++]=c;
                c = usart_receive();
            }
            c = usart_receive(); // also flush '\n' out of read buffer
            if(!failed){
                for(int i=5; i>=6-counter; i--){
                    conv_buffer[i] = conv_buffer[i - (6-counter)];
                }
                for(int i=0; i<(6-counter); i++){
                    conv_buffer[i] = '0';
                }
                moist_vars[k]=atoi(conv_buffer);
            }
            
			counter =0;
            while(usart_receive() != '"') { //scan input till you find ". The number will follow
               // c = getChar();
            }
            c=usart_receive();
            if(c=='F')
                failed=true;
            while(c != '"' && !failed){
                conv_buffer[counter++]=c;
                c = usart_receive();
            }
            c = usart_receive(); // also flush '\n' out of read buffer
            if(!failed){
                for(int i=5; i>=6-counter; i--){
                    conv_buffer[i] = conv_buffer[i - (6-counter)];
                }
                for(int i=0; i<(6-counter); i++){
                    conv_buffer[i] = '0';
                }
                tmp_vars[k]=atoi(conv_buffer);
            }
        }
        if(!failed) {
            lcd_clear();
            leds = 1;
            watering_pot=0;
            for(int k=0; k<middle_boards; ++k){//fix temperature!!!!!!
                sprintf(string_to_print, "T %d: %.1f ", k, tmp_avgs[k]);

                for(int m=0; m<strlen(string_to_print); ++m)
                    print(string_to_print[m]);

                if(moist_avgs[k]>=800){
                    strcpy(string_to_print, "VDRY ");
                    dry[k]=1;
                }
                else if(moist_avgs[k]>=600){
                    dry[k]=1;
                    strcpy(string_to_print, "DRY ");
                }
                else if(moist_avgs[k]>=370) {
                    dry[k]=0;
                    strcpy(string_to_print, "HUM ");
                }
                else{
                    dry[k]=0;
                    strcpy(string_to_print, "VHUM ");
                }

                for(int m=0; m<strlen(string_to_print); ++m)
                    print(string_to_print[m]);
                
                if(dry[k])
                    watering_pot = watering_pot | leds;
                leds = leds << 1;
            }
            PORTB = watering_pot;
        }
    }
    else
        first=false;
    lcd_clear();
    TCNT1 = 3036;
}


int main() {
	first = true;
	/*
    UBRRH = (ubrr_content >> 8); //set USART Baud Rate Register
    UBRRL = ubrr_content;

    //Receiver and Transmitter Enable, RX_interrupt enable, TX_interrupt enable

    UCSRB = (1 << TXEN) | (1 << TXCIE) | (1 << RXEN) | (1 << RXCIE);
    UCSRC = (1 << UCSZ1) | (1 << UCSZ0); //Char size(8 bits)*/
	
	usart_init(MYUBRR);
	usart_transmit('\n');
	
    for(int i=0; i<middle_boards; ++i) { //initialize_values
        moist_avgs[i]=0;
        tmp_avgs[i]=0.0;
        moist_vars[i]=0;
        tmp_vars[i]=0;
    }

    strcpy(string_to_send, "ESP:restart\n");
    sendCommand(string_to_send);
	
	usart_receive(); //wait until restart is complete
	while(UCSRA&(1<<RXC))
	usart_receive();
	
	strcpy(string_to_send, "ESP:ssid:\"Main_Board\"\n");
	sendCommand(string_to_send);

	/*
    strcpy(string_to_send, "ESP:addSensor:\"Moist_Sensor\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Tmp_Sensor\"\n");
    sendCommand(string_to_send);*/

    strcpy(string_to_send, "ESP:addSensor:\"Moist_avg\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Tmp_avg\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Moist_var\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Tmp_var\"\n");
    sendCommand(string_to_send);
	
	strcpy(string_to_send, "ESP:APStart\n");
	sendCommand(string_to_send);

    TCCR1B = 0x05; //CK/1024
	TCNT1 = 3036; //8 seconds
	TIMSK = 0x04; //enable overflow interrupt for TCNT1
    sei();

    while(1){}
}
