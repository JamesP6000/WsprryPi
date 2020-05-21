prefix=/usr/local

CFLAGS += -Wall -Wno-unused-variable 
CXXFLAGS +=  -Wall -Wall -Wno-unused-variable -std=c++11
LDLIBS += -lm



all: wspr gpioclk


wspr: mailbox.o wspr.cpp mailbox.h
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)  wspr.cpp librpitx/src/librpitx.a -owspr

gpioclk: gpioclk.cpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)  gpioclk.cpp -ogpioclk

clean:
	$(RM) *.o gpioclk wspr

.PHONY: install
install: wspr
	install -m 0755 wspr $(prefix)/bin
	install -m 0755 gpioclk $(prefix)/bin

.PHONY: uninstall
uninstall:
	$(RM) $(prefix)/bin/wspr $(prefix)/bin/gpioclk

