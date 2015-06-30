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

C4GOPEN_SRC_DIR = ../..
INCLUDES = -I$(LINUX_SRC_DIR)/include -I$(C4GOPEN_SRC_DIR)/include -I$(RTAI_INSTALL_DIR)/include -I$(RTNET_INSTALL_DIR)/include
LIBS = -lm $(RTAI_INSTALL_DIR)/lib/liblxrt.a -lpthread -L$(C4GOPEN_SRC_DIR)/lib -lC4gOpen

VERSION_NUMBER = $(shell ../../bin/getVersion --small)

EXECUTABLE=SimulinkMode4_server

default:
	$(CXX) $(CXXFLAGS) SimulinkMode4_server.cpp $(INCLUDES) $(LIBS) -DVERSION_NUMBER='"$(VERSION_NUMBER)"' -o $(EXECUTABLE)

install:
	@mkdir -p $(DESTDIR)/testsuite/SimulinkMode4_SERVER
	cp -f $(EXECUTABLE) $(DESTDIR)/testsuite/SimulinkMode4_SERVER

uninstall:
	@cd	$(DESTDIR)/testsuite/SimulinkMode4_SERVER && rm -f $(EXECUTABLE)

clean:
	rm -f $(EXECUTABLE)
