#pragma once



#include <modal_pipe_client.h>




typedef struct gps_data_t{
	double latitude;
	double longitude;
	double altitude;
} gps_data_t;

int pipe_clients_init(void);
gps_data_t grab_gps_info(void);
int grab_cpu_standby_active(void);

int pipe_clients_close(void);
