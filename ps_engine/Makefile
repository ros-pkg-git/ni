all: installed

GIT_DIR = build/ps_engine
GIT_URL = https://github.com/ros-pkg-git/Sensor.git
#GIT_PATCH = 
GIT_REVISION=ros

ENGINE=$(shell echo $$(eval rospack find ps_engine | sed 's/\//\\\//g'))
ENGINE_LIB=$(shell echo $$(eval rospack find ps_engine | sed 's/\//\\\//g')\\/lib)
ENGINE_BIN=$(shell echo $$(eval rospack find ps_engine | sed 's/\//\\\//g')\\/bin)
OPENNI_INCLUDE=$(shell echo $$(eval rospack find openni | sed 's/\//\\\//g')\\/include)
OPENNI_LIB=$(shell echo $$(eval rospack find openni | sed 's/\//\\\//g')\\/lib)
PLATFORM=Linux-x86

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	@-mkdir -p bin lib config 
	@echo "### Patching the main Makefile for PS Engine..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && \
		sed -i -e 's/CFG=$$(CFG)//g' Makefile && \
		cd -
	@echo "### Patching the output library for the core libraries..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB) -Wl,-rpath,$(ENGINE_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(ENGINE)/g' EngineLibMakefile && \
		sed -Ei -e 's/\/usr\/include\/ni\/CommonMakefile/$(OPENNI_INCLUDE)\/ni\/CommonMakefile/g' EngineLibMakefile && \
		sed -Ei -e 's/\/usr\/include/$(OPENNI_INCLUDE)/g' EngineLibMakefile && \
		cd -
	@echo "### Patching the output library for XnServer..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Utils && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB) -Wl,-rpath,$(ENGINE_LIB)\nLIB_DIRS=$(OPENNI_LIB) $(ENGINE_LIB)\nBIN_DIR=$(ENGINE)/g' EngineUtilMakefile && \
		sed -Ei -e 's/\/usr\/include\/ni\/CommonMakefile/$(OPENNI_INCLUDE)\/ni\/CommonMakefile/g' EngineUtilMakefile && \
		sed -Ei -e 's/\/usr\/include/$(OPENNI_INCLUDE)/g' EngineUtilMakefile && \
		cd -
	@-cd $(GIT_DIR)/Source/XnDeviceSensorV2 && \
		sed -Ei -e 's/\/usr\/bin/$(ENGINE_BIN)/g' XnSensorClient.cpp && \
		cd -
	cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make && cd -
	@cp -r $(GIT_DIR)/Data/GlobalDefaults.ini lib/
	for file in `cat $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/install.sh | grep MODULES= | sed 's/\"//g' | awk -F "=" {'print $$2'}`; do \
		LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$$file lib; \
	done
	touch installed
#	@$(foreach file,\
#		$(shell echo $$(eval cat $(GIT_DIR)/Platform/$(PLATFORM)/Redist/install.sh | grep MODULES= | sed 's/\"//g' | awk -F "=" {'print $$2'})),\
#		LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$(file) lib;)
#	$(foreach file,$(wildcard lib/*.so),LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r $(file) lib;)

clean:
	-cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make clean && cd -
	rm -rf $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/Final/*
	rm -f lib/*.so
	rm -rf bin config installed

wipe:
	rm -rf bin lib config installed
	rm -rf build patched rospack_nosubdirs
