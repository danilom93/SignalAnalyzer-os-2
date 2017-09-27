/*
 * This is a user-space application that reads /dev/sample
 * and prints the read characters to stdout
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#define	INVALID		-1
#define START		1
#define STOP		0

unsigned int 	read_val	= 0;
unsigned int 	state		= 1;
float 			period 		= 1;

void sigint_handler(void){
	
	state = 0;
	return;
}

int main(int argc, char **argv){

	char *app_name = argv[0];
	char *dev_name = "/dev/sample";
	int fd = -1;

	signal(SIGINT, sigint_handler);

	/*
 	 * Open the sample device RD | WR
 	 */
	if((fd = open(dev_name, O_RDWR)) < 0){

		fprintf(stderr, "%s: unable to open %s: %s\n", app_name, dev_name, strerror(errno));
		return(1);
	}
	printf("Start...\n");
	ioctl( fd, START, NULL );		
	while(state){
		
		read(fd, &read_val, sizeof(read_val));
		period = (float)read_val;
		printf("\33[2J");
		printf("\n\tSignal analyzer\n");
		printf("\n\n\tT = %5.5f [ms]  \tF = %5.5f [KHz]",period/1000, (1/period)*1000);
		printf("\n\n\tCtrl-c to exit\n");
		fflush(stdout);
		usleep(500000);
	}
	printf("\nStopping driver...\n");
	ioctl( fd, STOP, NULL );		   

	if(fd >= 0){

		close(fd);
	}
	return( 0 );
}
