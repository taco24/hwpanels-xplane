
LIBPATH=
WINDLLMAIN=
COMPILERFLAGS=-Wall
CC=gcc
#CC=gcc -v -da -Q
LIBRARIES=
HIDFILE=


HOSTOS=$(shell uname | tr A-Z a-z)
ifeq ($(HOSTOS),linux)
 LNFLAGS=
 CFLAGS=$(COMPILERFLAGS)
 LIBRARIES+=-lpthread -lusb-1.0 -ludev -lrt -lncurses
 HIDFILE=../linux/hid.c
 INCLUDE+=-I../linux/include
 INCLUDES+=-I`pkg-config libusb-1.0 --cflags`
else
  HOSTOS=windows
  LNFLAGS=-m32 -Wl,-O1 -L.
  CFLAGS=$(COMPILERFLAGS)
  LIBPATH+=-L".\SDK\Libraries\Win" -L".\lib"
  LIBRARIES+=-lsetupapi -lpthreadGC1
  INCLUDE+=-I./include
  HIDFILE=windows/hid.c
endif


all:
	$(CC) -c $(INCLUDE) $(CFLAGS) $(HIDFILE)
	$(CC) -c $(INCLUDE) $(CFLAGS) cb_driver.c
	$(CC) -c $(INCLUDE) $(CFLAGS) colomboard.c
	$(CC) -c $(INCLUDE) $(CFLAGS) commpanel.c
	$(CC) -c $(INCLUDE) $(CFLAGS) output.c
	$(CC) -c $(INCLUDE) $(CFLAGS) thread.c
	$(CC) -c $(INCLUDE) $(CFLAGS) time.c

	$(CC) -o commpanel hid.o cb_driver.o colomboard.o commpanel.o output.o thread.o time.o $(WINDLLMAIN) $(LNFLAGS) $(LIBPATH) $(LIBRARIES)

clean:
	$(RM) *.o commpanel
