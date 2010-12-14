all: nite

ARCH = $(shell uname -m)
ifeq ($(ARCH), i686)
ARCH_DIR = x86
else ifeq ($(ARCH), i486)
ARCH_DIR = x86
else ifeq ($(ARCH), i386)
ARCH_DIR = x86
else ifeq ($(ARCH), x86_64)
ARCH_DIR = x64
endif
BIN_DIR = $(SOURCE_DIR)/bin-pc/$(ARCH_DIR)


TARBALL     = build/NITE-Bin-Ubuntu-$(ARCH_DIR)-1.3.0.17.tar.bz2
TARBALL_URL = http://downloads.primesense.com/NITE-Bin-Ubuntu-$(ARCH_DIR)-1.3.0.17.tar.bz2
UNPACK_CMD  = tar xfj
SOURCE_DIR  = build/Nite-1.3.0.17
TARBALL_PATCH = 
MD5SUM_FILE = NITE-Bin-Ubuntu-$(ARCH_DIR)-1.3.0.17.tar.bz2.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = .
nite: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	mkdir -p $(INSTALL_DIR)/lib
	cp $(SOURCE_DIR)/Bin/libXnVNite.so $(INSTALL_DIR)/lib/
	mkdir -p $(INSTALL_DIR)/include/nite
	cp $(SOURCE_DIR)/Include/* $(INSTALL_DIR)/include/nite
	cp $(SOURCE_DIR)/XnVFeatures/Bin/lib*.so $(INSTALL_DIR)/lib/
	mkdir -p $(INSTALL_DIR)/XnVFeatures
	cp $(SOURCE_DIR)/XnVFeatures/Data/* $(INSTALL_DIR)/XnVFeatures
	rosrun openni niReg $(INSTALL_DIR)/lib/libXnVFeatures.so $(INSTALL_DIR)/XnVFeatures
	cp $(SOURCE_DIR)/XnVHandGenerator/Bin/lib*.so $(INSTALL_DIR)/lib/
	mkdir -p $(INSTALL_DIR)/XnVHandGenerator
	cp $(SOURCE_DIR)/XnVHandGenerator/Data/* $(INSTALL_DIR)/XnVHandGenerator
	rosrun openni niReg $(INSTALL_DIR)/lib/libXnVHandGenerator.so $(INSTALL_DIR)/XnVHandGenerator
	rosrun openni niLicense PrimeSense 0KOIk2JeIBYClPWVnMoRKn5cdY4=
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) build $(INSTALL_DIR)/lib $(INSTALL_DIR)/include $(INSTALL_DIR)/XnVFeatures $(INSTALL_DIR)/XnVHandGenerator

wipe: clean
