//
// demo application to send MAVLink open drone ID commands 
// to a MAVLink capable Open Drone ID transponder
//
// (c) Bluemark Innovations BV 
// MIT license
//
// The code has been tested with the BlueMark DroneBeacon MAVLink
//  transponder, where an USB-to-UART adapter was connected to
// /dev/ttyUSB1 and 9600 baud rate. 
//
// if you want to change this, please UART_PORT and UART_BAUDRATE accordingly
// on line 40/41
 
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>

#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <sys/ioctl.h>

#include <common/mavlink.h>
#include <opendroneid.h>
#include <mav2odid.h>

#define MAVLINK_SYSTEM_ID       3
#define MAVLINK_COMPONENT_ID    1

#define UART_PORT 				"/dev/ttyUSB1"
#define UART_BAUDRATE			B9600

pthread_mutex_t  serial_port_lock;
int uart0_FS_RDWR = -1;

static uint64_t current_timestamp_ms() 
{   //return current timestamp in milliseconds
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    uint64_t milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
        
    return milliseconds;
}

//print mavlink functions copied from opendroneid-core-c/test/test_mav2odid.c
static void print_mavlink_auth(mavlink_open_drone_id_authentication_t *auth)
{
    printf("Data page: %d, auth type: %d, ",
           auth->data_page, auth->authentication_type);
    int size = MAVLINK_MSG_OPEN_DRONE_ID_AUTHENTICATION_FIELD_AUTHENTICATION_DATA_LEN;
    if (auth->data_page == 0) {
        size = ODID_AUTH_PAGE_ZERO_DATA_SIZE;
        printf("last_page_index: %d, length: %d, timestamp: %u, ",
               auth->last_page_index, auth->length, auth->timestamp);
    }
    printf("\n");
    for (int i = 0; i < size; i++)
        printf("0x%02X ", auth->authentication_data[i]);
    printf("\n");
}

static void print_mavlink_operatorID(mavlink_open_drone_id_operator_id_t *operator_id)
{
    // Ensure the ID is null-terminated
    char buf[MAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_FIELD_OPERATOR_ID_LEN + 1] = { 0 };
    memcpy(buf, operator_id->operator_id, MAVLINK_MSG_OPEN_DRONE_ID_OPERATOR_ID_FIELD_OPERATOR_ID_LEN);

    printf("Type: %d, operator ID: %s\n",
           operator_id->operator_id_type, buf);
}

static void print_mavlink_location(mavlink_open_drone_id_location_t *location)
{
    printf("Status: %d, Direction: %d cdeg, SpeedHori: %d cm/s, SpeedVert: %d cm/s, \n"\
           "Lat/Lon: %d, %d degE7, Alt: Baro, Geo, Height above %s: %.2f, %.2f, %.2f\n"\
           "Horiz, Vert, Baro, Speed, TS Accuracy: %d, %d, %d, %d, %d, TimeStamp: %.2f\n",
           location->status, location->direction, location->speed_horizontal,
           location->speed_vertical, location->latitude, location->longitude,
           location->height_reference ? "Ground" : "TakeOff",
           (double) location->altitude_barometric,
           (double) location->altitude_geodetic, (double) location->height,
           location->horizontal_accuracy, location->vertical_accuracy,
           location->barometer_accuracy, location->speed_accuracy,
           location->timestamp_accuracy, (double) location->timestamp);
}


static void print_mavlink_system(mavlink_open_drone_id_system_t *system)
{
    printf("Operator Location Type: %d, Classification Type: %d\n"
           "Lat/Lon: %d, %d degE7, \n"
           "Area Count, Radius: %d, %d, Ceiling, Floor: %.2f, %.2f m\n"
           "Category EU: %d, Class EU: %d\n"
           "Operator Altitude Geodetic: %.2f\n",
           system->operator_location_type, system->classification_type,
           system->operator_latitude, system->operator_longitude,
           system->area_count, system->area_radius,
           (double) system->area_ceiling, (double) system->area_floor,
           system->category_eu, system->class_eu, (double) system->operator_altitude_geo);
}

static void print_mavlink_selfID(mavlink_open_drone_id_self_id_t *self_id)
{
    // Ensure the description is null-terminated
    char buf[MAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_FIELD_DESCRIPTION_LEN + 1] = { 0 };
    memcpy(buf, self_id->description, MAVLINK_MSG_OPEN_DRONE_ID_SELF_ID_FIELD_DESCRIPTION_LEN);

    printf("Type: %d, description: %s\n",
           self_id->description_type, buf);
}


static void print_mavlink_basicID(mavlink_open_drone_id_basic_id_t *basic_id)
{
    // Ensure the ID is null-terminated
    char buf[MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_UAS_ID_LEN + 1] = { 0 };
    memcpy(buf, basic_id->uas_id, MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_UAS_ID_LEN);

    printf("ID type: %d, UA type: %d, UAS ID: %s\n",
           basic_id->id_type, basic_id->ua_type, buf);
}

//send OpenDrone ID messages to Remote ID transponder
void MAVLink_send_open_drone_ID_messages()
{	
	if( uart0_FS_RDWR != -1 )
	{		
		mavlink_message_t msg = { 0 };
		uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
		memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);
		int tx_bytes;
		int message_bytes;

		//OpenDrone ID basic message
		mavlink_open_drone_id_basic_id_t basic_id = {
			.ua_type = MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR,
			.id_type = 	ODID_IDTYPE_UTM_ASSIGNED_UUID };

		uint8_t uas_id[] = "9876543210ABCDEFGHJK";
		memcpy(basic_id.uas_id, uas_id, sizeof(basic_id.uas_id));

		printf("\n--------------------------Basic ID message------------------------\n\n");
		print_mavlink_basicID(&basic_id);
		memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);
		mavlink_msg_open_drone_id_basic_id_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &basic_id);
		
		message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);
																		
		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}
		
		usleep(25000); //sleep some time to allow transponder to receive and decode message
		
		//OpenDrone ID self ID message
		mavlink_open_drone_id_self_id_t selfID = {
			.description_type = MAV_ODID_DESC_TYPE_TEXT,
			.description = "MAVLink test flight" };

		printf("\n\n------------------------Self ID------------------------\n\n");
		print_mavlink_selfID(&selfID);

		mavlink_msg_open_drone_id_self_id_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &selfID);
		message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);
		
		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}

		usleep(25000); //sleep some time to allow transponder to receive and decode message
		
		//OpenDrone ID system message
		mavlink_open_drone_id_system_t system = {
			.operator_location_type = MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF,
			.classification_type = MAV_ODID_CLASSIFICATION_TYPE_EU,			
			.operator_latitude = (int32_t) (52.2564348 * 1E7),
			.operator_longitude = (int32_t) (6.8296092 * 1E7),
			.area_count = 1,
			.area_radius = 0,
			.area_ceiling = -1000,
			.area_floor = -1000, 
			.category_eu = MAV_ODID_CATEGORY_EU_CERTIFIED,
			.class_eu = MAV_ODID_CLASS_EU_CLASS_5,
			.operator_altitude_geo = 1.3f,
			.timestamp = (unsigned)time(NULL) - 1546300800 };

		printf("\n\n------------------------System------------------------\n\n");
		print_mavlink_system(&system);
		
		memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);
		mavlink_msg_open_drone_id_system_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &system);
		message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);

		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}
		
		//OpenDrone ID operator message
		mavlink_open_drone_id_operator_id_t operatorID = {
			.operator_id_type = MAV_ODID_OPERATOR_ID_TYPE_CAA };
			
		char operator_id[] = "ABCDEFGHJK0123456789";
		memcpy(operatorID.operator_id, operator_id, sizeof(operatorID.operator_id));

		printf("\n\n----------------------Operator ID-----------------------\n\n");
		print_mavlink_operatorID(&operatorID);

		mavlink_msg_open_drone_id_operator_id_encode(MAVLINK_SYSTEM_ID,	MAVLINK_COMPONENT_ID, &msg, &operatorID);
		message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);        

		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);		
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}

		usleep(25000); //sleep some time to allow transponder to receive and decode message

		memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);
		mavlink_msg_open_drone_id_operator_id_encode(MAVLINK_SYSTEM_ID,	MAVLINK_COMPONENT_ID, &msg, &operatorID);

		message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);  
		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);	
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}
		
			
		printf("\n\n---------------------Authentication---------------------\n\n");		
		
		//generate auth message containing 3 pages
		mavlink_open_drone_id_authentication_t auth0 = {
			.data_page = 0,
			.authentication_type = MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE,			
			.last_page_index = 2,
			.length = 17,
			.timestamp = 23000000 };
				
		memset(auth0.authentication_data,1,ODID_AUTH_PAGE_ZERO_DATA_SIZE); //set data to all 1s
		
		mavlink_open_drone_id_authentication_t auth1 = {
			.data_page = 1,	
			.authentication_type = MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE };										
		
		memset(auth1.authentication_data,2,ODID_AUTH_PAGE_NONZERO_DATA_SIZE); //set data to all 2s
						
		mavlink_open_drone_id_authentication_t auth2 = {
			.data_page = 2,
			.authentication_type = MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE };
						
		memset(auth2.authentication_data,3,ODID_AUTH_PAGE_NONZERO_DATA_SIZE); //set data to all 3s
		
		print_mavlink_auth(&auth0); //print page 0
		print_mavlink_auth(&auth1); //print page 1
		print_mavlink_auth(&auth2); //print page 2
    
		
        memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);        
        mavlink_msg_open_drone_id_authentication_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &auth0);                

        message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);  
		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);	
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}
		
		usleep(25000); //sleep some time to allow transponder to receive and decode message
		
		memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);        
        mavlink_msg_open_drone_id_authentication_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &auth1);                

        message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);  
		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);	
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}
		
		usleep(25000); //sleep some time to allow transponder to receive and decode message
		
		memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);        
        mavlink_msg_open_drone_id_authentication_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &auth2);                

        message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);  
		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);	
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}
		
		//OpenDrone ID location message
		unsigned int ts_odid_hour = (unsigned)time(NULL) % (60*60*24)/3600; //only hour part of current time
		unsigned int ts_odid_minute = (unsigned)time(NULL) % (60*60)/60; //only minute part

		mavlink_open_drone_id_location_t location = {
			.status = MAV_ODID_STATUS_AIRBORNE,
			.direction = (uint16_t) 15,
			.speed_horizontal = (uint16_t) 350,
			.speed_vertical = (int16_t) 100,
			.latitude = (int32_t) (52.2364348 * 1E7),
			.longitude = (int32_t) (6.8496092 * 1E7),
			.altitude_barometric = 37.5,
			.altitude_geodetic = 36.5,
			.timestamp = ts_odid_hour*60 + ts_odid_minute,
			.height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF,
			.height = 55.0,
			.horizontal_accuracy = MAV_ODID_HOR_ACC_3_METER,
			.vertical_accuracy = MAV_ODID_VER_ACC_1_METER,
			.barometer_accuracy = MAV_ODID_VER_ACC_3_METER,
			.speed_accuracy = MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND,
			.timestamp_accuracy = MAV_ODID_TIME_ACC_0_1_SECOND
		};		

		printf("\n\n------------------------Location------------------------\n\n");
		print_mavlink_location(&location);
		memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);
		mavlink_msg_open_drone_id_location_encode(MAVLINK_SYSTEM_ID,
		MAVLINK_COMPONENT_ID,
		&msg, &location);
		message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);		

		pthread_mutex_lock(&serial_port_lock);
		tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);
		pthread_mutex_unlock(&serial_port_lock);
		if (tx_bytes != message_bytes)
		{
			printf("ERROR: UART TX error\n");
		}

		usleep(25000); //sleep some time to allow transponder to receive and decode message
	}
	else
	{
		printf("could not open serial interface!\n");
	}

	
}

//thread to parse received MAVLink messages and send every second/1 Hz heartbeat messages. 
void *serial_port_receive(void *ptr) 
{
	uint8_t rx_buffer[MAVLINK_MAX_PACKET_LEN];
	uint64_t time_last_heartbeat_ms = 0;
	
	while(1)
	{		
		union {
			mavlink_heartbeat_t heartbeat;			
		} msg_rx;
							
		usleep(100000);
		
		memset(rx_buffer,0,MAVLINK_MAX_PACKET_LEN);
		pthread_mutex_lock(&serial_port_lock);
		int rx_length = read(uart0_FS_RDWR, (void*)rx_buffer, MAVLINK_MAX_PACKET_LEN);
		pthread_mutex_unlock(&serial_port_lock);
		
		if (rx_length > 0)
		{				
			rx_buffer[rx_length] = '\0';			
			mavlink_message_t message;			
			mavlink_status_t status;
			int i = 0;
			while (i < rx_length)
			{
				mavlink_message_t message;			
				mavlink_status_t status;
				
				if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[i++], &message, &status))
				{						
					switch ((int) message.msgid)
					{
						case MAVLINK_MSG_ID_HEARTBEAT:							
							mavlink_msg_heartbeat_decode(&message, &msg_rx.heartbeat);
							//printf("heartbeat ver 0x%X status 0x%X autopilot 0x%X type 0x%X\n",msg_rx.heartbeat.mavlink_version, msg_rx.heartbeat.system_status, msg_rx.heartbeat.autopilot, msg_rx.heartbeat.type);
							printf("#");
							fflush(stdout);
							break;														
						default:
							printf("unknown message\n");
							printf("received MAVLink message: 0x%X len %i | flags 0x%X 0x%X seq %i sysid 0x%X compid 0x%X msgid %i\n",message.magic,message.len,message.incompat_flags,message.compat_flags,message.seq,message.sysid,message.compid,message.msgid);
							break;
					}
				}
			}		
		}
		
		//transmit heart beat message at 1 Hz
		uint64_t time_ms = current_timestamp_ms(); 
        if ((time_ms - time_last_heartbeat_ms) > (1000 - 25)) //1Hz
        {
			time_last_heartbeat_ms = time_ms;
			
			//send new heart beat message
			mavlink_heartbeat_t heartbeat;
			heartbeat.type = MAV_TYPE_ODID;
			heartbeat.autopilot = MAV_AUTOPILOT_INVALID; //for not flight controll
			heartbeat.base_mode = 0;
			heartbeat.custom_mode = 0;
			heartbeat.system_status = MAV_STATE_ACTIVE;
		
			mavlink_message_t msg = { 0 };	
			mavlink_msg_heartbeat_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &heartbeat);

			uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
			memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);			
			int message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);
			
			pthread_mutex_lock(&serial_port_lock);	
			int tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);
			pthread_mutex_unlock(&serial_port_lock);
			if (tx_bytes != message_bytes)
			{
				printf("ERROR: UART TX error\n");
			} 
		}
	}
}
	
int main(int argc, char *argv[] )
{
	int result = pthread_mutex_init(&serial_port_lock, NULL);
	
	pthread_t thread_serial_port_RX;
	int  iret_serial_port_RX;
	char *message_serial_port_RX = "Thread receiving MAVLink messages";
	
	char interface_UART[128]; //interface to use
	strcpy(interface_UART,UART_PORT);
	
	if( access(interface_UART, F_OK ) != -1 )
	{
		uart0_FS_RDWR = open(interface_UART, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
		if (uart0_FS_RDWR == -1)
		{
			printf("WARNING: Unable to open UART interface for MAVLink communication. %s\n",interface_UART);
			exit(11);
		}
		struct termios options;
		tcgetattr(uart0_FS_RDWR, &options);
		options.c_cflag = UART_BAUDRATE | CS8 | CLOCAL | CREAD;		
		options.c_iflag = IGNPAR;
		options.c_oflag = 0;
		options.c_lflag = 0;

		options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
		options.c_oflag &= ~(ONLCR | OCRNL);
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

		tcflush(uart0_FS_RDWR, TCIFLUSH);
		tcsetattr(uart0_FS_RDWR, TCSANOW, &options);						
	}
	
	//set timezone to application
	char buf[80];
	snprintf(buf, sizeof(buf), "TZ=UTC");
	putenv(buf);
	tzset();

	char *date2 = __DATE__;
	char *time2 = __TIME__;
	printf("MAVlink OpenDrone ID simulator to test MAVLink transponders\n2022 Bluemark Innovations\n");
	printf("version: %s %s\n", date2,time2);

	iret_serial_port_RX = pthread_create( &thread_serial_port_RX, NULL, serial_port_receive,(void*) message_serial_port_RX);
			
	while (1)
	{
		MAVLink_send_open_drone_ID_messages(); // send message every 10 seconds
		sleep(10);
	}
	
	return 0;
}
