#include <SoftwareSerial.h>

#define rxPin 10
#define txPin 11
#define  recv 0
#define  ok  1
#define standby 2
#define debug
//#define debug_print_reg
#define default_init_baud 9600
#define QUERY_BAUND 0x00
#define MAX_MESSAGE_LENGTH 100
#define REQ_LENGTH_BASIS 6
#define MODBUS_MAX_READ_REGISTERS          125
#define _MIN_REQ_LENGTH 12
#define MAX_MESSAGE_LENGTH 100
#define MODBUS_MAX_WRITE_REGISTERS         123
#define MODBUS_MAX_RW_WRITE_REGISTERS      121
#define _MODBUS_RTU_HEADER_LENGTH      1


/* Function codes */
#define _FC_READ_COILS                0x01
#define _FC_READ_DISCRETE_INPUTS      0x02
#define _FC_READ_HOLDING_REGISTERS    0x03
#define _FC_READ_INPUT_REGISTERS      0x04
#define _FC_WRITE_SINGLE_COIL         0x05
#define _FC_WRITE_SINGLE_REGISTER     0x06
#define _FC_READ_EXCEPTION_STATUS     0x07
#define _FC_WRITE_MULTIPLE_COILS      0x0F
#define _FC_WRITE_MULTIPLE_REGISTERS  0x10
#define _FC_REPORT_SLAVE_ID           0x11
#define _FC_WRITE_AND_READ_REGISTERS  0x17


#define _POLL_INPUT_REGISTERS  	0x01
#define _POLL_HOLDING_REGISTERS	0x02
#define _POLL_WRITE_REGISTERS		0x03

#define MAX_QUEUE 8
#define MAX_UNSIG_LONG 0xFFFFFFFF

SoftwareSerial Wifi_Serial(rxPin, txPin);

unsigned char req[_MIN_REQ_LENGTH];
unsigned char  rsp[MAX_MESSAGE_LENGTH];
volatile unsigned char Wifi_uart_baud = 4;
unsigned char  read_usb_status = standby;
unsigned char  read_wifi_status = standby;
unsigned int dest[10];
int wifi_length = 0;
unsigned long  long pre_time = 0, last_time = 0;
int rsp_length = 0x0;
int JUDGE_OK = 1;
unsigned char timeout_times = 1;
unsigned int DELAY_TIMEOUT = 5000;
int poll_status = _POLL_INPUT_REGISTERS;
unsigned int  write_reg_addr = 0,write_reg_value=0;
unsigned long  write_out_buffer[MAX_QUEUE];
unsigned int  DC_Volt =0,DC_Cur=0,DC_Volt_gain=0,DC_Cur_gain=0;
unsigned char queue_ptn =0 ;
unsigned char write_times =0;

void updata_uart_parameter()
{
	long baud;

	switch (Wifi_uart_baud)
	{
		case 1: baud = 1200; break;
		case 2: baud = 2400; break;
		case 3: baud = 4800; break;
		case 4: baud = 9600; break;
		case 5: baud = 14400; break;
		case 6: baud = 19200; break;
		case 7: baud = 38400; break;
		case 8: baud = 57600; break;
		case 9: baud = 115200; break;
		default : baud = 1200; break;
	}

	Wifi_Serial.begin(baud);
	Wifi_Serial.listen();

}

void setup() 
{
	pinMode(rxPin, INPUT);
	pinMode(txPin, OUTPUT);
	updata_uart_parameter();
	
	Serial.begin(9600);
	
	memset(dest, 0, sizeof(dest));
	
	memset(req, 0, sizeof(req));
	
	memset(rsp, 0, sizeof(rsp));
	
	memset(write_out_buffer,0,sizeof(write_out_buffer));

	delay(100); 

	if (QUERY_BAUND)
	{
		send_Serial(Wifi_Serial, "+++");
		pre_time = millis();
	}
	
	//void queue_in(unsigned int addr,unsigned int val)
	// queue_in(0x0E,0x01);
	// queue_in(0x0F,0x01);
	write_reg_addr = 0x0E;
	write_reg_value = 0x01;

}



void loop() {
	 last_time = millis();
	 
	if (Wifi_Serial.available())
	{
	#ifdef debug
		Serial.print("read_serial_data\n");
	#endif
		pre_time = millis();
		read_wifi_status = recv;
		read_Serial();
		delay(2);
	}
 last_time = millis();
	 
	if (read_wifi_status == ok)
	{
		if(JUDGE_OK == 0)
		{
			#ifdef debug
			Serial.print("judge_data_deal");
			#endif
			judge_data();
			
		}else{
			
			recv_buffer_deal();
			delay(50);
			modbus_poll();
		}
		
		 read_wifi_status = standby;
		 wifi_length = 0;
		 
	}
	
 last_time = millis();
 
	 if(Serial.available())
	{
		 read_usb_status = recv;
		 delay(2);
	}

	 if(read_usb_status == ok)
	{
		 read_usb_status = standby;
	}

 last_time = millis();
	if( last_time - pre_time > DELAY_TIMEOUT)
	{

		if(timeout_times < 1000)
		{
			 Serial.print("enter timeout subroutine:");
			 Serial.print(timeout_times); 
			 Serial.print('\n'); 
			 
			 judge_model();
			 timeout_times++;
		}else{
			
			DELAY_TIMEOUT = 10000 ;
			timeout_times = 0;
			
		}
		
		 pre_time = millis();
	}		

 last_time = millis();
}

void judge_model()
{
	if(JUDGE_OK==1)
	{
		auto_login_cmd();

		#ifdef debug
		Serial.print("judge_to_cmd\n");
		#endif
	}
	else
	{
		#ifdef debug
		Serial.print("judge_to_acc_uart\n");
		#endif
		
		acc_uart();
	}

		
}

void read_Serial()
{

	unsigned char ch;

	 if(wifi_length < 100)
	{
		ch = Wifi_Serial.read();

	if(ch >0)
		rsp[wifi_length]=ch;
	
	wifi_length ++;
	delay(1);

	if(!Wifi_Serial.available()) 
		read_wifi_status = ok ;
	else
		read_Serial();
	}

}


void  judge_data()
{
	 rsp[wifi_length]='\0';

	 if(strcmp((const char *)rsp,"a") == 0)
	{

		 Serial.write("judge ok baud is: ");
		 Serial.println(Wifi_uart_baud);
		 memset(rsp,0,sizeof(rsp));
		 
		 auto_login_cmd();

		 
		 Wifi_Serial.flush();
		 delay(10);
		 Wifi_Serial.print("hello");
		 Wifi_Serial.flush();
		 delay(10);
		 
		 modbus_poll();

	}else{

		#ifdef debug
		 Serial.write("judge data recv : ");
		 Serial.write((const char*)rsp);
		 Serial.write("wifi_length \n");
		 Serial.print(wifi_length);
		 Serial.write('\n');
		#endif 

		 acc_uart();

	}
}

 void acc_uart()
{

	#ifdef debug
	 Serial.write("enter change baud subroutine ");
	 Serial.write('\n');
	#endif

	 if(Wifi_uart_baud< 9)
		Wifi_uart_baud++;
	 else
		Wifi_uart_baud = 1;

	 delay(400);
	 updata_uart_parameter();
	 send_Serial(Wifi_Serial,"+++");

	#ifdef debug
	 Serial.write( "chang baud subroutine Wifi_uart_baud is : ");
	 Serial.print(Wifi_uart_baud);
	 Serial.write('\n');
	#endif
}

 void send_Serial(SoftwareSerial s,const char  *string)
{
	s.write(string);
}


 void auto_login_cmd()
{
	#ifdef debug
	Serial.print("auto_login_cmd\n");
	#endif

	if(JUDGE_OK == 0)
		JUDGE_OK=1;
	
	modbus_poll();
	
}


 static const unsigned char table_crc_hi[] = {0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

 static const unsigned char table_crc_lo[] = {0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D,0xCD,0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,0x78, 0xB8, 0xB9,0x79,0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,0x70,0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94,0x54, 0x9C, 0x5C,0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

unsigned int crc16(unsigned char  *buffer,unsigned int buffer_length)
{
	 unsigned char crc_hi = 0xFF; /* high CRC byte initialized */
	 unsigned char  crc_lo = 0xFF; /* low CRC byte initialized */
	 unsigned int i; /* will index into CRC lookup */

	 /* pass through message buffer */
	 while (buffer_length--) {
	 i = crc_hi ^ *buffer++; /* calculate the CRC  */
	 crc_hi = crc_lo ^ table_crc_hi[i];
	 crc_lo = table_crc_lo[i];
	}

	 return (crc_hi << 8 | crc_lo);
}



 int build_req_basis(int function,int addr,int nb,unsigned char *req)
{
	 req[0] = 0x1;
	 req[1] = function;
	 req[2] = addr >> 8;
	 req[3] = addr & 0x00ff;
	 req[4] = nb >> 8;
	 req[5] = nb & 0x00ff;
	 return REQ_LENGTH_BASIS ;
}


 int send_msg_pre(unsigned  char *req,int req_length)
{
	 unsigned int crc = crc16(req, req_length);
	 req[req_length++] = crc >> 8;
	 req[req_length++] = crc & 0x00FF;

	 return req_length;
}

int modbus_write_register(int addr, int value)
{
    return write_single(_FC_WRITE_SINGLE_REGISTER, addr, value);
}

/* Write a value to the specified register of the remote device.
   Used by write_bit and write_register */
static int write_single(int function, int addr, int value)
{
    int rc;
    int req_length;

    req_length = build_req_basis(function, addr, value, req);
	req_length = send_msg_pre(req,req_length);
    rc = send_msg(req, req_length);
		
    return rc;
}

 int modbus_read_registers( int addr, int nb)
{
	 int status;

	 if (nb > MODBUS_MAX_READ_REGISTERS) {
		return -1;
	}

	 status = read_registers( _FC_READ_HOLDING_REGISTERS,addr, nb);
	 return status;
}

 int modbus_read_input_registers(int addr, int nb)
{
	 int status;

	 if(nb>MODBUS_MAX_READ_REGISTERS)
	{
		return -1;
	}

	 status = read_registers(_FC_READ_INPUT_REGISTERS,addr,nb);
	 return status;
}

 int read_registers( int function, int addr, int nb)
{
	 int rc;
	 int req_length;


	 if (nb > MODBUS_MAX_READ_REGISTERS) {
		return -1;
	}

	 req_length = build_req_basis(function, addr, nb, req);
	 req_length=send_msg_pre(req,req_length);
	 rc = send_msg(req, req_length);
		
	 return rc;
}

 int send_msg(unsigned char *req,int  req_length)
{
	#ifdef debug_read_write
	Serial.print("send_msg\n");
	#endif
	
	 int i=0;
	 int rc=0;
	 for(i=0;i<req_length;i++)
	 {
		 rc += Wifi_Serial.write(req[i]);
		 Wifi_Serial.flush();
		 //Serial.print(req[i]);
	 }

	//Wifi_Serial.setTimeout(500);
	
	 return rc;
}

int check_confirmation(unsigned char *req,unsigned char *rsp, int rsp_length)
{
	#ifdef debug
	Serial.print("check_confirmation\n");
	#endif
	
	 int rc=-1;
	 int rsp_length_computed=0;
	 const int offset = 1;


	 rsp_length_computed = compute_response_length_from_request(req);

	 #ifdef debug
	Serial.print("rsp_length_computed:");
	Serial.println(rsp_length_computed);
	Serial.print("rsp_length:");
	Serial.println(rsp_length);
	#endif
	
	
	 /* Check length */
	 if (rsp_length == rsp_length_computed )
	{
		#ifdef debug
		Serial.print("check_confirmation_rsp_length==rsp_length_computed\n");
		#endif
		
		 int req_nb_value;
		 int rsp_nb_value;
		 const int function = rsp[offset];

		 /* Check function code */
		 if (function != req[offset]) 
		{
			return -1;
		}

	 /* Check the number of values is corresponding to the request */
		 switch (function) 
		 {
			 case _FC_READ_COILS:
			 case _FC_READ_DISCRETE_INPUTS:
			 /* Read functions, 8 values in a byte (nb
			 * of values in the request and byte count in
			 * the response. */
			 req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			 req_nb_value = (req_nb_value / 8) + ((req_nb_value % 8) ? 1 : 0);
			 rsp_nb_value = rsp[offset + 1];
			 break;
			 case _FC_WRITE_AND_READ_REGISTERS:
			 case _FC_READ_HOLDING_REGISTERS:
			 case _FC_READ_INPUT_REGISTERS:
			 /* Read functions 1 value = 2 bytes */
			 req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			 rsp_nb_value = (rsp[offset + 1] / 2);
			 break;
			 case _FC_WRITE_MULTIPLE_COILS:
			 case _FC_WRITE_MULTIPLE_REGISTERS:
			 /* N Write functions */
			 req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			 rsp_nb_value = (rsp[offset + 3] << 8) | rsp[offset + 4];
			 break;
			 case _FC_REPORT_SLAVE_ID:
			 /* Report slave ID (bytes received) */
			 req_nb_value = rsp_nb_value = rsp[offset + 1];
			 break;
			 default:
			 /* 1 Write functions & others */
			 req_nb_value = rsp_nb_value = 1;
		}

		 if (req_nb_value == rsp_nb_value) {
		 rc = rsp_nb_value;
		} else {
		 rc = -1;
		}
	} 

 return rc;
}


unsigned int compute_response_length_from_request(unsigned char *req)
{
	#ifdef debug
	Serial.print("compute_response_length_from_request\n");
	#endif
	
	 int length;
	 const int offset = 1;

	 switch (req[offset]) {
	 case _FC_READ_COILS:
	 case _FC_READ_DISCRETE_INPUTS: {
		 /* Header + nb values (code from write_bits) */
		 int nb = (req[offset + 3] << 8) | req[offset + 4];
		 length = 2 + (nb / 8) + ((nb % 8) ? 1 : 0);
		}
		 break;
		 case _FC_WRITE_AND_READ_REGISTERS:
		 case _FC_READ_HOLDING_REGISTERS:
		 case _FC_READ_INPUT_REGISTERS:
		 /* Header + 2 * nb values */
		 length = 2 + 2 * (req[offset + 3] << 8 | req[offset + 4]);
		 break;
		 case _FC_READ_EXCEPTION_STATUS:
		 length = 3;
		 break;
	 default:
	 length = 5;
	}

	 return   offset+length + 2;
}


int modbus_poll( )
{
	int  nb =0;
	
	#ifdef debug
	Serial.print("poll_status:\t");
	Serial.print(poll_status);
	Serial.print('\n');
	#endif
	
	switch(poll_status)
	{
		case  _POLL_INPUT_REGISTERS:
		{
			// int modbus_read_input_registers(int addr, int nb, unsigned char *dest)
			nb = modbus_read_input_registers(0x0,0x02);
			
			break;
		}
		case  _POLL_HOLDING_REGISTERS:
		{
			// int modbus_read_registers( int addr, int nb, unsigned char *dest)
			nb =  modbus_read_registers(0x9,0x02);
			
			break;
		}
		case  _POLL_WRITE_REGISTERS:
		{
			//int modbus_write_register(int addr, int value)
			// int write_reg_addr int write_reg_value
			if(write_reg_addr != 0 && write_reg_value !=0)
 				nb =  modbus_write_register(write_reg_addr,write_reg_value);
			
			break;
		}
		
		default:
			break;
	}
	
	return nb;
}


int write_registers_deal(int rsp_length)
{
	int rc = -1;
	
	if( rsp[0] != 0)
	{
		rc = check_confirmation(req, rsp, rsp_length);
		memset(rsp,0,sizeof(rsp));
		
	}else{
	
		return -1;
		
	}
	
	return rc;
}

int input_holding_registers_deal(int rsp_length)
{
	int rc = -1;
	int i = 0;
	
	if( rsp[0] != 0)
	{
		
		rc = check_confirmation(req, rsp, rsp_length);

		 if (rc == -1)
			return -1;

		 for (i = 0; i < rc; i++)
		{

			 /* shift reg hi_byte to temp OR with lo_byte */
			 dest[i] = (rsp[3 + (i << 1)]<<8 ) |rsp[4 + (i << 1)] ;
		}
			rsp_length = 0;
			
		memset(rsp,0,sizeof(rsp));
		
	}else{
		
		return -1;
	}
	
	return rc;
}


void  recv_buffer_deal()
{
	int nb = 0;
	#ifdef debug_print_reg
	unsigned char i= 0;
	#endif
	
	switch(poll_status)
	{
		case _POLL_HOLDING_REGISTERS:
		{
			nb = input_holding_registers_deal(wifi_length);
			
			#ifdef debug
			Serial.println("poll_status _POLL_HOLDING_REGISTERS");
			// Serial.print("poll_status _nb:");
			// Serial.print(nb);
			#endif
			
			if(nb) 
			 {
			 
			 #ifdef debug_print_reg
				 for(i=0;i<nb;i++)
				{
					 Serial.print("Read holding Registers ");
					 Serial.print(i);
					 Serial.print(": ");
					 Serial.println(dest[i]);
					 
				}	
				#endif
				
				if(DC_Volt_gain != dest[0] || DC_Cur_gain != dest[1])
				{
					Serial.println("queue_in");
					
					DC_Volt_gain = dest[0];
					DC_Cur_gain = dest[1];
					
					if(write_times  < 10)
					{
						Serial.print("write_times:");
						Serial.println(write_times);
						
						queue_in(0x09,++DC_Volt_gain);
						queue_in(0x0A,++DC_Cur_gain);
						write_times ++;
						
					}
					
				}else if((write_times>=10)&&(DC_Volt_gain!=10000||DC_Cur_gain!=10400)){
				
						Serial.println("orgin_data_write");
						
						queue_in(0x09,10000);
						queue_in(0x0A,10400);
						
				}
				
				Serial.print("DC_Volt_gain:");
				Serial.print(DC_Volt_gain);
				Serial.print('\n');
				
				Serial.print("DC_Cur_gain:");
				Serial.print(DC_Cur_gain);
				Serial.print('\n');
			
				  //input_reg_poll_data = ++dest[0];
				
				pre_time = millis();
				
				memset(dest,0,sizeof(dest));
				
				poll_status = _POLL_WRITE_REGISTERS;
			 }
			 
			break;
		}
		case _POLL_INPUT_REGISTERS:
		{
			#ifdef debug
			Serial.print("poll_status _POLL_INPUT_REGISTERS\n");
			#endif
			
			nb = input_holding_registers_deal(wifi_length);
			
			if(nb) 
			 {
			 #ifdef debug_print_reg
				 for(i=0;i<nb;i++)
				{
					 Serial.print("Read Input Registers ");
					 Serial.print(i);
					 Serial.print(": ");
					 Serial.print(dest[i]);
					 Serial.print('\n');
				}
				#endif
				
				DC_Volt = dest[0];
				DC_Cur = dest[1];
				
				Serial.print("DC_Volt:");
				Serial.print(DC_Volt);
				Serial.print('\n');
				
				Serial.print("DC_Cur:");
				Serial.print(DC_Cur);
				Serial.print('\n');
				
				pre_time = millis();
				memset(dest,0,sizeof(dest));
				poll_status = _POLL_HOLDING_REGISTERS;
			 }
			 
			break;
		}
		case _POLL_WRITE_REGISTERS:
		{
			#ifdef debug
			Serial.print("poll_status _POLL_WRITE_REGISTERS\n");
			#endif
			
			nb = write_registers_deal(wifi_length);
			
			if(nb != -1)
			{
				poll_status = _POLL_INPUT_REGISTERS;
				get_queue_data_to_write();
				
			}
		
			break;
		}
	}
}


void queue_in(unsigned int addr,unsigned int val)
{
	unsigned long tmp = long(addr);
	
	if(queue_ptn < MAX_QUEUE)
		write_out_buffer[queue_ptn++] = (tmp<<16 ) | val;
		
	// unsigned int a= 0,b=0;
	// a = write_out_buffer[queue_ptn-1] >>16;
 	// Serial.println(a,HEX);
	// b = write_out_buffer[queue_ptn-1] &0x0000ffff;
	// Serial.println(b,HEX);
	
	// Serial.print("queue_in:");
	// Serial.print(write_out_buffer[queue_ptn-1],HEX);
	// Serial.print('\n');
	
}

unsigned long queue_out()
{
	unsigned char i=0;
	
	if(queue_ptn != 0)
	{
		unsigned long tmp = write_out_buffer[0];
		
		if((queue_ptn < MAX_QUEUE) )
		{	
			if(queue_ptn > 1)
				for(i=0;i<queue_ptn;i++)
				{
					write_out_buffer[i] = write_out_buffer[i+1];
				}
		
			write_out_buffer[i-1] = 0;
			
			queue_ptn--;
		}
		Serial.print("queue_data:");
		Serial.println(tmp,HEX);
		return tmp;
		
	}else{
		
		return MAX_UNSIG_LONG;
	}
}

void get_queue_data_to_write()
{
	unsigned long ret_val = queue_out();
	
	if(ret_val != MAX_UNSIG_LONG)
	{
		write_reg_addr	=	ret_val >> 16;
		write_reg_value	= ret_val & 0x0000ffff;
		
		Serial.print("get_queue_data_to_write:");
		Serial.print(write_reg_addr);
		Serial.print(":");
		Serial.print(write_reg_value);
		Serial.print('\n');
		
	}else{
	
		write_reg_addr = 0;
		write_reg_value = 0;
	}

}

void optimize_buffer_sort()
{
	
}


