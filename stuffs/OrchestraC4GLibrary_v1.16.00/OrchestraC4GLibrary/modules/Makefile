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
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC cpp/C4GSensor.cpp -d ./cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC cpp/C4GActuatorMode1.cpp -d ./cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC cpp/C4GActuatorMode2.cpp -d ./cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC cpp/C4GActuatorMode4.cpp -d ./cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC cpp/C4GActuatorMode5.cpp -d ./cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC cpp/C4GActuatorMode7.cpp -d ./cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC cpp/C4GActuatorMode8.cpp -d ./cpp --includes="$(INCLUDES)" --libs="$(LIBS)"
	$(ORCHESTRACORE_INSTALL_DIR)/bin/makeDLC cpp/C4GActuatorMode9.cpp -d ./cpp --includes="$(INCLUDES)" --libs="$(LIBS)"

install:
	@mkdir -p $(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/modules/dlc
	cp -f cpp/*.dlc $(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/modules/dlc
	@mkdir -p $(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/modules/xmi
	cp -f xmi/*.xmi $(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/modules/xmi

uninstall:
	@cd	$(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/modules/dlc && rm -f *.dlc
	@cd	$(C4GLIBRARY_INSTALL_DIR)/OrchestraC4GLibrary/modules/xmi && rm -f *.xmi

clean:
	rm -f cpp/*.dlc
