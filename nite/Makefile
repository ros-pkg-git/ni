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

NITE_LIB=$(shell echo $$(eval rospack find nite | sed 's/\//\\\//g')\\/lib)

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
	cd $(INSTALL_DIR)/XnVHandGenerator && sed -Ei -e 's/^;//g' Nite.ini
	rosrun openni niReg $(INSTALL_DIR)/lib/libXnVHandGenerator.so $(INSTALL_DIR)/XnVHandGenerator
	rosrun openni niLicense PrimeSense 0KOIk2JeIBYClPWVnMoRKn5cdY4=
	-cp $(SOURCE_DIR)/Data/*.xml $(INSTALL_DIR)/lib
	@-cd $(SOURCE_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/Data\/Sample-Tracking.xml/$(NITE_LIB)\/Sample-Tracking.xml/g' Boxes/main.cpp
	@-cd $(SOURCE_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/Data\/Sample-Tracking.xml/$(NITE_LIB)\/Sample-Tracking.xml/g' CircleControl/main.cpp
	@-cd $(SOURCE_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/Data\/Sample-User.xml/$(NITE_LIB)\/Sample-User.xml/g' Players/main.cpp
	@-cd $(SOURCE_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/Data\/Sample-Tacking.xml/$(NITE_LIB)\/Sample-Tracking.xml/g' PointServer/main.cpp
	@-cd $(SOURCE_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/Data\/Sample-Tracking.xml/$(NITE_LIB)\/Sample-Tracking.xml/g' PointViewer/main.cpp
	@-cd $(SOURCE_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/Data\/Sample-Scene.xml/$(NITE_LIB)\/Sample-Scene.xml/g' SceneAnalysis/main.cpp
	@-cd $(SOURCE_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/Data\/Sample-Tracking.xml/$(NITE_LIB)\/Sample-Tracking.xml/g' SingleControl/main.cpp
	@-cd $(SOURCE_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/Data\/Sample-Tracking.xml/$(NITE_LIB)\/Sample-Tracking.xml/g' TrackPad/main.cpp
	@-sed -Ei -e 's/<MapOutputMode xRes="320" yRes="240" FPS="30"\/>/<MapOutputMode xRes="640" yRes="480" FPS="30"\/>/g' lib/Sample-User.xml
	@-sed -Ei -e 's/<MapOutputMode xRes="320" yRes="240" FPS="30"\/>/<MapOutputMode xRes="640" yRes="480" FPS="30"\/>/g' lib/Sample-Scene.xml
	@-sed -Ei -e 's/<MapOutputMode xRes="320" yRes="240" FPS="30"\/>/<MapOutputMode xRes="640" yRes="480" FPS="30"\/>/g' lib/Sample-Tracking.xml
	cd $(SOURCE_DIR)/; make INC_DIRS="$(shell rospack find openni)/include/ni $(shell rospack find nite)/include/nite" LIB_DIRS_OPTION="-L$(shell rospack find openni)/lib/ -L$(shell rospack find nite)/lib -Xlinker -R$(shell rospack find openni)/lib/ -Xlinker -R$(shell rospack find nite)/lib/"
	mkdir -p $(INSTALL_DIR)/bin
	cp $(SOURCE_DIR)/Samples/Bin/* $(INSTALL_DIR)/bin
	touch $(INSTALL_DIR)/installed



clean:
	rm -rf $(SOURCE_DIR) build $(INSTALL_DIR)/bin $(INSTALL_DIR)/lib $(INSTALL_DIR)/include $(INSTALL_DIR)/XnVFeatures $(INSTALL_DIR)/XnVHandGenerator

wipe: clean
