/*
    C socket server
*/
#include <time.h> 
#include<stdio.h>
#include<string.h>    //strlen
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include<unistd.h>    //write
 
int getClient()
{
    int socket_desc , client_sock , c , read_size, index=0, bindStatus;
    int sizeofportNum = 5;
    int portNum[5] = { 8888, 7777, 6666, 5555, 4444};
    struct sockaddr_in server , client;
    
    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
        printf("Could not create socket");
    }
    puts("Socket created");
     
    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    while(index< sizeofportNum)
    {
	    server.sin_port = htons( portNum[index] );
  
	     //Bind
	    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    		bindStatus = 0;

	    else
		{
			bindStatus = 1;
			printf("Accessed Port:%d \n",portNum[index]);
			break;
		}
	    index = index + 1;
    }

    if (bindStatus==0)
    {
	//print the error message
      	perror("You have exceeded the number of interrupted disconnections.\n Please try again after 60 seconds.");
        return 1;
    }

    puts("bind done");
     
    //Listen
    listen(socket_desc , 1);
     
    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    c = sizeof(struct sockaddr_in);
     
    //accept connection from an incoming client
    client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
    if (client_sock < 0)
    {
        perror("accept failed");
        return 1;
    }
    puts("Connection accepted");
    
    return client_sock;
}