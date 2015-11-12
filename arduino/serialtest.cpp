
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

int main( int argc, char **argv )
{
    if ( argc != 2 )
    {
        fprintf( stderr, "usage: %s <tty path>\n", argv[0] );
        return 1;
    }
    
    int fd = open(argv[1],O_RDWR|O_NONBLOCK);
    if ( fd < 0 )
    {
        fprintf( stderr, "could not open %s for read/write; errno: %d\n", argv[1], errno );
        return 1;
    }
    
    for ( int i = 0; i < 10; i++ )
    {
        unsigned char data[2];
        int totalread = 0;
        while ( totalread < 2 )
        {
            int nread = read( fd, data+totalread, 2-totalread );
            if ( nread < 0 )
            {
                if ( errno == EAGAIN )
                {
                    //usleep(100);
                } else {
                    fprintf( stderr, "error: read() failed; errno: %d\n", errno );
                    close(fd);
                    return 1;
                }
            }
            else
            {
                totalread += nread;
            }
        }
        fprintf( stdout, "values: %d %d\n", data[0], data[1] );
    }
    
    close(fd);
}