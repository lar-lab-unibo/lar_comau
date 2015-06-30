ifndef C4GLIBRARY_INSTALL_DIR
	C4GLIBRARY_INSTALL_DIR = /usr/local/C4Gopen
endif

ifndef LINUX_SRC_DIR
	LINUX_SRC_DIR = /usr/src/linux
endif

ORCHESTRACORE_INSTALL_DIR = /usr/local/Orchestra/Core
RTNET_INSTALL_DIR = /usr/local/rtnet

INCLUDES = -I$(LINUX_SRC_DIR)/include -I$(C4GLIBRARY_INSTALL_DIR)/include -I$(RTNET_INSTALL_DIR)/include
LIBS = -L$(C4GLIBRARY_INSTALL_DIR)/lib -lC4gOpen

default:
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC Sensor.cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC Generator.cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC Actuator.cpp --includes="$(INCLUDES)" --libs="$(LIBS)"

install:
	@mkdir -p $(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/examples/Mode5
	cp -f *.xcl *.xmi *.xmp *.dlc $(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/examples/Mode5

uninstall:
	@cd	$(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/examples/Mode5 && rm -f *.xcl *.xmi *.xmp *.dlc

clean:
	rm -f *.dlc
