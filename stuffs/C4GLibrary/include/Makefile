ifndef DESTDIR
	DESTDIR = /usr/local/C4Gopen
endif

install:
	@mkdir -p $(DESTDIR)/include
	cp C4gOpenConstants.hpp C4gOpen.hpp $(DESTDIR)/include

uninstall:
	rm -f $(DESTDIR)/include/C4gOpenConstants.hpp
	rm -f $(DESTDIR)/include/C4gOpen.hpp

