/* client.c */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <errno.h>

#define SERVER_PORT 5040
#define BUFLEN      255

int main(int argc, char **argv) 
{
  int s, rc, i;
  struct sockaddr_in cliAddr, remoteServAddr;
  struct hostent *h;
  char buf[BUFLEN] = "Hello world!";

  /* Check for i.p. address input */
  if(argc < 1) 
  {
    printf ("Usage: %s <server> \n", argv[0]);
    exit(EXIT_FAILURE);
  }

  /* Compare with server's i.p. */
  h = gethostbyname(argv[1]);
  if(h == NULL) 
  {
    printf("%s: Unknown host '%s' \n", argv[0], argv[1]);
    exit(EXIT_FAILURE);
  }

  printf("%s: Sending data to '%s' (IP : %s) \n",
     argv[0], h->h_name,
     inet_ntoa(*(struct in_addr *) h->h_addr_list[0]));
  
  remoteServAddr.sin_family = h->h_addrtype;
  memcpy((char *) &remoteServAddr.sin_addr.s_addr,
           h->h_addr_list[0], h->h_length);
  remoteServAddr.sin_port = htons(SERVER_PORT);
  
  /* Create Socket */
  s = socket(AF_INET, SOCK_DGRAM, 0);
  if(s < 0) 
  {
     printf("%s: Can't open Socket (%s) \n", 
     	    argv[0], strerror(errno));
     exit(EXIT_FAILURE);
  }

  /* Bind with every port */
  cliAddr.sin_family = AF_INET;
  cliAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  cliAddr.sin_port = htons(0);
  rc = bind(s, (struct sockaddr *) &cliAddr,
              sizeof(cliAddr));
  if(rc < 0) 
  {
     printf("%s: Can't bind with Port (%s)\n",
        argv[0], strerror(errno));
     exit(EXIT_FAILURE);
  }

  /* Send data */
	rc = sendto(s, buf, strlen(buf), 0,
               (struct sockaddr *) &remoteServAddr,
                sizeof (remoteServAddr));
    if(rc < 0) 
    {
       printf ("%s: Can't send data %d\n",argv[0], i-1);
       close(s);
       exit (EXIT_FAILURE);
    }
  return EXIT_SUCCESS;
}