ifndef DESTDIR
	DESTDIR = /usr/local/C4Gopen
endif

ifndef RTAI_INSTALL_DIR
	RTAI_INSTALL_DIR = /usr/realtime
endif

ifndef RTNET_INSTALL_DIR
	RTNET_INSTALL_DIR = /usr/local/rtnet
endif

ifndef LINUX_SRC_DIR
	LINUX_SRC_DIR = /usr/src/linux
endif

ifndef CXX
	CXX = g++
endif

ifndef CXXFLAGS
	CXXFLAGS = -O2 -Wall
endif

C4GOPEN_SRC_DIR = ..
INCLUDES = -I$(LINUX_SRC_DIR)/include -I$(C4GOPEN_SRC_DIR)/include -I$(RTAI_INSTALL_DIR)/include -I$(RTNET_INSTALL_DIR)/include

default:
	$(CXX) $(CXXFLAGS) -c C4gOpen.cpp $(INCLUDES) -o libC4gOpen.a

install:
	@mkdir -p $(DESTDIR)/lib
	cp -f libC4gOpen.a $(DESTDIR)/lib
	
uninstall:
	rm -f $(DESTDIR)/lib/libC4gOpen.a

clean:
	rm -f *.o
	rm -f *.a

