#MS VC: cl lpc21iap.c elf.c lpcusb.c libusb.lib

LIBNAME	= usb
APPNAME = lpc21iap

# Package definitions
PKG_NAME	= lpc21iap
DATE		= $$(date +%Y%m%d)
DATEL		= $$(date +%Y%m%d-%H%M)

# Tool definitions
CC      = gcc
LD      = ld
AR      = ar
AS      = as
CP      = objcopy
OD		= objdump
RM		= rm
TAR		= tar

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
  FPIC=-fPIC
else
  FPIC=
endif

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
  LIBRARYS = $(shell if test -d /opt/local/lib; then echo "-L/opt/local/lib"; elif test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; fi)
  INCLUDES = $(shell if test -d /opt/local/include; then echo "-I/opt/local/include"; elif test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; fi)
endif

CFLAGS  = -Wall -g $(FPIC) $(INCLUDES) $(LIBRARYS)

SOURCES =	lpc21iap.c elf.c lpcusb.c

all: $(APPNAME)

clean:
	$(RM) -f core $(APPNAME).o $(APPNAME) $(APPNAME).obj $(APPNAME).exe

app: $(APPNAME)

$(APPNAME):	$(SOURCES) Makefile
	$(CC) $(CFLAGS) $(SOURCES) -o $(APPNAME) -l$(LIBNAME)

# Builds archive tar file
arch: clean
	cd .. && $(TAR) --exclude={CVS,cvs} -cvzf $(PKG_NAME)-$(DATEL).tar.gz $(PKG_NAME)

# Builds release tar file
dist: clean
	cd .. && $(TAR) --exclude={CVS,cvs} -cvzf $(PKG_NAME)-$(DATE).tar.gz $(PKG_NAME)
