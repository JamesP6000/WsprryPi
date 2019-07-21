prefix=/usr/local

CFLAGS += -Wall
CXXFLAGS += -D_GLIBCXX_DEBUG -std=c++14 -Wall -Werror -Wno-psabi
LDLIBS += -lm

ifeq ($(findstring armv6,$(shell uname -m)),armv6)
# Broadcom BCM2835 SoC with 700 MHz 32-bit ARM 1176JZF-S (ARMv6 arch)
PI_VERSION = -DRPI1
else
# Broadcom BCM2836 SoC with 900 MHz 32-bit quad-core ARM Cortex-A7  (ARMv7 arch)
# Broadcom BCM2837 SoC with 1.2 GHz 64-bit quad-core ARM Cortex-A53 (ARMv8 arch)
PI_VERSION = -DRPI23
endif

all: wspr gpioclk

mailbox.o: mailbox.c mailbox.h
	$(CC) $(CFLAGS) -c mailbox.c

nhash.o: nhash.c nhash.h
	$(CC) $(CFLAGS) -c nhash.c

wspr: mailbox.o nhash.o wspr.cpp mailbox.h
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(LDLIBS) $(PI_VERSION) mailbox.o nhash.o wspr.cpp -o wspr

gpioclk: gpioclk.cpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(LDLIBS) $(PI_VERSION) gpioclk.cpp -o gpioclk

clean:
	$(RM) *.o gpioclk wspr

.PHONY: install
install: wspr
	install -m 0755 wspr $(prefix)/bin
	install -m 0755 gpioclk $(prefix)/bin

.PHONY: uninstall
uninstall:
	$(RM) $(prefix)/bin/wspr $(prefix)/bin/gpioclk

