// sconio.h - subset of conio.h
// Calls Microsoft conio.h or implements subset of conio.h functions not implemented in GCC

#ifndef __SCONIO_H__
#define __SCONIO_H__

#if _MSC_VER
#include <conio.h>
#else                                               // If compiler not MSVC then include the function subset

// This kbhit () copied from : http://cboard.cprogramming.com/linux-programming/51531-faq-cached-input-mygetch.html#post357655

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/time.h>

#define _kbhit kbhit

int kbhit (void);

int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;
 
  tv.tv_sec = 0;
  tv.tv_usec = 0;
 
  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);
 
  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);
}

// These functions copied from here : http://stackoverflow.com/questions/3276546/how-to-implement-getch-function-of-c-in-linux

#define _getch getch
#define _getche getche

/* reads from keypress, doesn't echo */
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

/* reads from keypress, echoes */
int getche(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

#endif
#endif
