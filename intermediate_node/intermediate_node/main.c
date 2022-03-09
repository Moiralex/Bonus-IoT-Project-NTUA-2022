#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define f_osc 8000000
#define baud  9600
#define ubrr_content ((f_osc/(16*baud))-1)
#define tx_buffer_size 128
#define rx_buffer_size 128
#define sensor_boards 1

#define FOSC 8000000 // Clock Speed gia thn seiriakh
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

char rx_buffer[rx_buffer_size];
uint8_t rx_ReadPos = 0;
uint8_t rx_WritePos = 0;

char tx_buffer[tx_buffer_size];
char string_to_send[tx_buffer_size];
char conv_buffer[6]; //variance is at max 1024^2/4=262144 which has 6 digits
uint8_t tx_ReadPos = 0;
uint8_t tx_WritePos = 0;

int moistures[sensor_boards];
int temperatures[sensor_boards];
int moist_avg, moist_var, tmp_var;
float tmp_avg, tmp_var_f;
bool first;

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
    int counter =0;

    if(!first) {
        for(int k=0; k<sensor_boards; ++k){
			counter = 0;
            failed=false;
            /*strcpy(string_to_send, "ESP:ssid:\"Sens_Board");
            itoa(k, conv_buffer, 10);
            strcat(string_to_send, conv_buffer);
            strcat(string_to_send, "\"\n");*/
            sprintf(string_to_send, "ESP:ssid:\"Sens_Board%d\"\n", k);
            sendCommand(string_to_send);

            strcpy(string_to_send, "ESP:password:\"awesomePassword\"\n");
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:sensorValue:\"Moist_Sensor\"[request]\n");
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:sensorValue:\"Tmp_Sensor\"[request]\n");
            sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:connect\n");
            sendCommand(string_to_send);
            //strcpy(string_to_send, "ESP:clientTransmit\n");
            //sendCommand(string_to_send);
            strcpy(string_to_send, "ESP:getAllValues\n");
            serialWrite(string_to_send);
            //c=getChar();
            while(usart_receive() != '"' && !failed) { //scan input till you find ". The number will follow
               // c = getChar();
            }
            c=usart_receive(); //read most significant digit
            if(c=='F')
                failed=true;
            while(c != '"' && !failed){ // read the whole number (until " is read)
                conv_buffer[counter++]=c;
                c = usart_receive();
            }
            c = usart_receive(); // also flush '\n' out of read buffer
            if(!failed){ 
                for(int i=5; i>=6-counter; i--){ // place number at the end of the buffer and fill the start of it with 0s so that it can be converted to an int
                    conv_buffer[i] = conv_buffer[i - (6-counter)];
                }
                for(int i=0; i<(6-counter); i++){
                    conv_buffer[i] = '0';
                }
                moistures[k]=atoi(conv_buffer);
            }
            

            //c=getChar();
			counter = 0;
            while(usart_receive() != '"' && !failed) {
                //c = getChar();
            }
            c = usart_receive();
            if(c=='F')
                failed=true;
            while(c != '"' && !failed){
                conv_buffer[counter++]=c;
                c = usart_receive();
            }
            c = usart_receive(); // also flush '\n' out of read buffer
            if(!failed){
                for(int i=5; i>=6-counter; i--){ // place number at the end of the buffer and fill the start of it with 0s so that it can be converted to an int
                    conv_buffer[i] = conv_buffer[i - (6-counter)];
                }
                for(int i=0; i<(6-counter); i++){
                    conv_buffer[i] = '0';
                }
                temperatures[k]=atof(conv_buffer);
            }
            
        }
        tmp_avg=0.0;
        moist_avg=0;
        for(int i=0; i<sensor_boards; ++i) {
            moist_avg += moistures[i];
            tmp_avg += temperatures[i];
        }
        moist_avg /= sensor_boards;
        tmp_avg /= sensor_boards;

        for(int i=0; i<sensor_boards; ++i) {
            moist_var += (moistures[i]-moist_avg)*(moistures[i]-moist_avg);
            tmp_var_f += (temperatures[i]-tmp_avg)*(temperatures[i]-tmp_avg);
        }
        moist_var /= sensor_boards;
        tmp_var_f /= sensor_boards;
        tmp_var = (int)tmp_var_f;

        //strcpy(string_to_send, "ESP:ssid:\"Middle_Board1\"\n");
        //sendCommand(string_to_send);

        /*itoa(moist_avg, conv_buffer, 10); //convert value calculated to string to send it to ESP
        strcpy(string_to_send, "ESP:sensorValue:\"Moist_avg\"["); //create the string to send to set the sensor value
        strcat(string_to_send, conv_buffer);
        strcat(string_to_send, "]\n");*/
        sprintf(string_to_send, "ESP:sensorValue:\"Moist_avg\"[%d]\n", moist_avg);
        sendCommand(string_to_send); //send command to set the value of the sensor

        /*itoa(tmp_avg, conv_buffer, 10); //convert value calculated to string to send it to ESP
        strcpy(string_to_send, "ESP:sensorValue:\"Tmp_avg\"["); //create the string to send to set the sensor value
        strcat(string_to_send, conv_buffer);
        strcat(string_to_send, "]\n");*/
        sprintf(string_to_send, "ESP:sensorValue:\"Tmp_avg\"[%.1f]\n", tmp_avg);
        sendCommand(string_to_send); //send command to set the value of the sensor

        /*itoa(moist_var, conv_buffer, 10); //convert value calculated to string to send it to ESP
        strcpy(string_to_send, "ESP:sensorValue:\"Moist_var\"["); //create the string to send to set the sensor value
        strcat(string_to_send, conv_buffer);
        strcat(string_to_send, "]\n");*/
        
        sprintf(string_to_send, "ESP:sensorValue:\"Moist_var\"[%d]\n", moist_var);
        sendCommand(string_to_send); //send command to set the value of the sensor

        /*itoa(tmp_var, conv_buffer, 10); //convert value calculated to string to send it to ESP
        strcpy(string_to_send, "ESP:sensorValue:\"Tmp_var\"["); //create the string to send to set the sensor value
        strcat(string_to_send, conv_buffer);
        strcat(string_to_send, "]\n");*/
        sprintf(string_to_send, "ESP:sensorValue:\"Tmp_var\"[%d]\n", tmp_var);
        sendCommand(string_to_send); //send command to set the value of the sensor

        //strcpy(string_to_send, "ESP:APStart\n");
        //sendCommand(string_to_send);
		strcpy(string_to_send, "ESP:clientTransmit\n");
		sendCommand(string_to_send);
     
    }
    else
        first=false;

    TCNT1 = 3036;
}


int main(){
	first=true;
	/*
    UBRRH = (ubrr_content >> 8); //set USART Baud Rate Register
    UBRRL = ubrr_content;

    //Receiver and Transmitter Enable, RX_interrupt enable, TX_interrupt enable
    UCSRB = (1 << TXEN) | (1 << TXCIE) | (1 << RXEN) | (1 << RXCIE);
    UCSRC = (1 << UCSZ1) | (1 << UCSZ0); //Char size(8 bits)*/
	
	usart_init(MYUBRR);
	
    for(int i=0; i<sensor_boards; ++i) { //initialize moistures and temperatures
        moistures[i]=0;
        temperatures[i]=0;
    }

    strcpy(string_to_send, "ESP:restart\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Moist_Sensor\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Tmp_Sensor\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Moist_avg\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Tmp_avg\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Moist_var\"\n");
    sendCommand(string_to_send);

    strcpy(string_to_send, "ESP:addSensor:\"Tmp_var\"\n");
    sendCommand(string_to_send);


    TCCR1B = 0x05; //CK/1024
	TCNT1 = 3036; //8 seconds
	TIMSK = 0x04; //enable overflow interrupt for TCNT1
    sei();

    while(1){}
}
