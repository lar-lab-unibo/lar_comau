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

EXECUTABLE=TestMode1

default:
	$(CXX) $(CXXFLAGS) TestMode1.cpp $(INCLUDES) $(LIBS) -DVERSION_NUMBER='"$(VERSION_NUMBER)"' -o $(EXECUTABLE)

install:
	@mkdir -p $(DESTDIR)/testsuite/TestMode1
	cp -f $(EXECUTABLE) $(DESTDIR)/testsuite/TestMode1

uninstall:
	@cd	$(DESTDIR)/testsuite/TestMode1 && rm -f $(EXECUTABLE)

clean:
	rm -f $(EXECUTABLE)
