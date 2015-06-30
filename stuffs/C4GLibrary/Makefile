export DESTDIR = /usr/local/C4Gopen
export RTAI_INSTALL_DIR = /usr/realtime
export RTNET_INSTALL_DIR = /usr/local/rtnet
export LINUX_SRC_DIR = /usr/src/linux
export CXX = g++
export CXXFLAGS = -O2 -Wall

default:
	@cd lib; make
	@cd testsuite; make
	@echo -e '\nType \140sudo make install\140 to install libC4gOpen in $(DESTDIR).\n'

debug: CXXFLAGS = -O0 -g -Wall
debug: default 

install:
	@whoami | grep -v root >/dev/null 2>/dev/null; if [ $$? -eq 0 ]; then echo "ERROR: you must be root (or use sudo)!"; exit 1; else exit 0; fi
	@cd lib && make install
	@cd include && make install
	@cd testsuite && make install
	@mkdir -p $(DESTDIR)/bin && cd bin && cp -f loadRTAI remRTAI loadRTnet remRTnet $(DESTDIR)/bin
	@mkdir -p $(DESTDIR)/Documentation && cd Documentation && cp -rf html $(DESTDIR)/Documentation
	@echo -e '\nlibC4gOpen installed in $(DESTDIR).\n'

uninstall:
	@whoami | grep -v root >/dev/null 2>/dev/null; if [ $$? -eq 0 ]; then echo "ERROR: you must be root (or use sudo)!"; exit 1; else exit 0; fi
	@cd lib && make uninstall
	@cd include && make uninstall
	@cd testsuite && make uninstall
	@cd $(DESTDIR)/bin && rm -f loadRTAI remRTAI loadRTnet remRTnet
	@cd $(DESTDIR)/Documentation && rm -rf html
	@bash -c 'if [ $$(expr `find $(DESTDIR)/ -type f -print | wc -l` + `find $(DESTDIR)/ -type l -print | wc -l`) == 0 ]; then rm -rf $(DESTDIR); fi'
	@echo -e '\nlibC4gOpen uninstalled from $(DESTDIR).\n'

clean:
	@cd lib && make clean
	@cd testsuite && make clean
	@echo -e '\nC4GLibrary source tree cleaned.\n'
