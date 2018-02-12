/* server.c */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#define LOCAL_SERVER_PORT 5040
#define BUF 255

int main(int argc, char **argv) 
{
  int s, rc, message, len;
  struct sockaddr_in cliAddr, servAddr;
  char buffer[BUF];
  time_t time1;
  char loctime[BUF];
  char *ptr;
  const int y = 1;
  
  /* Create a socket */
  s = socket(AF_INET, SOCK_DGRAM, 0);
  if(s < 0) 
  {
     printf("%s: Can't open socket...(%s)\n",
        argv[0], strerror(errno));
     exit(EXIT_FAILURE);
  }

  /* Bind Local Server Port */
  servAddr.sin_family = AF_INET;
  servAddr.sin_addr.s_addr = htonl (INADDR_ANY);
  servAddr.sin_port = htons (LOCAL_SERVER_PORT);
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &y, sizeof(int));
  rc = bind(s, (struct sockaddr *) &servAddr,
              sizeof(servAddr));
  if(rc < 0) 
  {
     printf("%s: Can't bind with Port number %d (%s)\n",
        argv[0], LOCAL_SERVER_PORT, strerror(errno));
     exit(EXIT_FAILURE);
  }
  printf("-------------------------------------\n");
  printf("--------------UDP Server-------------\n");
  printf("-------------------------------------\n");
  printf("%s: Waiting for data on Port (UDP) %u\n",
     argv[0], LOCAL_SERVER_PORT);
  
  /* Server Loop */
  while(1) 
  {
    /* Initialize Buffer */
    memset(buffer, 0, BUF);
    /* Receive messages */
    len = sizeof(cliAddr);
    message = recvfrom(s, buffer, BUF, 0,
                   (struct sockaddr *) &cliAddr, &len );
    if(message < 0) 
    {
       printf("%s: Can't receive data...\n",
          argv[0]);
       continue;
    }
    /* Print received message */
    if(message > 0) 
	{
		buffer[message] = 0;
		printf("Received message: %s\n", buffer);
	}
  }
  return EXIT_SUCCESS;
}