/*
**A simple UDP server
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define SERVICE_PORT 5040  //Port number
#define BUFSIZE      2048  //Length of buffer

int
main(int argc, char **argv)
{
	struct sockaddr_in myaddr;           //Our address 
	struct sockaddr_in remaddr;	     //Remote address
	socklen_t addrlen = sizeof(remaddr); //Length of addresses
	int recvlen;			     //No. of bytes received
	int fd;				     //Our socket
	unsigned char buf[BUFSIZE];	     //Receive buffer 


	//Create a UDP socket
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
	{
		perror("Cannot create socket\n");
		return 0;
	}

	//Bind the socket to any valid IP address and a specific port
	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(SERVICE_PORT);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) 
	{
		perror("Bind failed");
		return 0;
	}

	//Print received data
	for (;;) 
	{
		printf("Waiting on port %d\n", SERVICE_PORT);
		recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
		printf("Received %d bytes\n", recvlen);
		if (recvlen > 0) 
		{
			buf[recvlen] = 0;
			printf("Received message: \"%s\"\n", buf);
		}
	}
	//Never exits
}
