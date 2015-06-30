export C4GLIBRARY_INSTALL_DIR = /usr/local/C4Gopen
export LINUX_SRC_DIR = /usr/src/linux

default:
	@cd modules && make
	@cd examples && make
	@echo -e '\nType \140sudo make install\140 to install OrchestraC4GLibrary in $(C4GLIBRARY_INSTALL_DIR).\n'

install:
	@whoami | grep -v root >/dev/null 2>/dev/null; if [ $$? -eq 0 ]; then echo "ERROR: you must be root (or use sudo)!"; exit 1; else exit 0; fi
	@cd modules && make install
	@cd examples && make install
	@echo -e '\nOrchestraC4GLibrary installed in $(C4GLIBRARY_INSTALL_DIR).\n'

uninstall:
	@whoami | grep -v root >/dev/null 2>/dev/null; if [ $$? -eq 0 ]; then echo "ERROR: you must be root (or use sudo)!"; exit 1; else exit 0; fi
	@cd modules && make uninstall
	@cd examples && make uninstall
	@echo -e '\nOrchestraC4GLibrary uninstalled from $(C4GLIBRARY_INSTALL_DIR).\n'

clean:
	@cd modules && make clean
	@cd examples && make clean
	@echo -e '\nOrchestraC4GLibrary source tree cleaned.\n'
	