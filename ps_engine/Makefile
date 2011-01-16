all: installed

GIT_DIR = build/ps_engine
GIT_URL = https://github.com/ros-pkg-git/Sensor.git
#GIT_PATCH = 
# ros branch as of 2011-01-05
GIT_REVISION=894cea018698e49dc7dba02cc42d07a8e8f1cb69

SEDCOM=sed
LIBEXT=so
LD_VAR=LD_LIBRARY_PATH
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
  SEDCOM=gsed
  LIBEXT=dylib
  LD_VAR=DYLD_LIBRARY_PATH
endif

ENGINE=$(shell echo $$(eval rospack find ps_engine | $(SEDCOM) 's/\//\\\//g'))
ENGINE_LIB=$(shell echo $$(eval rospack find ps_engine | $(SEDCOM) 's/\//\\\//g')\\/lib)
ENGINE_BIN=$(shell echo $$(eval rospack find ps_engine | $(SEDCOM) 's/\//\\\//g')\\/bin)
OPENNI_INCLUDE=$(shell echo $$(eval rospack find openni | $(SEDCOM) 's/\//\\\//g')\\/include)
OPENNI_LIB=$(shell echo $$(eval rospack find openni | $(SEDCOM) 's/\//\\\//g')\\/lib)
PLATFORM=Linux-x86

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	@-mkdir -p bin lib config 
	@echo "### Patching the main Makefile for PS Engine..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && \
		$(SEDCOM) -i -e 's/CFG=$$(CFG)//g' Makefile && \
		cd -
	@echo "### Patching the output library for the core libraries..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB) -Wl,-rpath,$(ENGINE_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(ENGINE)/g' EngineLibMakefile && \
		$(SEDCOM) -i -e 's/\/usr\/include\/ni\/CommonMakefile/$(OPENNI_INCLUDE)\/ni\/CommonMakefile/g' EngineLibMakefile && \
		$(SEDCOM) -i -e 's/\/usr\/include/$(OPENNI_INCLUDE)/g' EngineLibMakefile && \
		cd -
	@echo "### Patching the output library for XnServer..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Utils && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB) -Wl,-rpath,$(ENGINE_LIB)\nLIB_DIRS=$(OPENNI_LIB) $(ENGINE_LIB)\nBIN_DIR=$(ENGINE)/g' EngineUtilMakefile && \
		$(SEDCOM) -i -e 's/\/usr\/include\/ni\/CommonMakefile/$(OPENNI_INCLUDE)\/ni\/CommonMakefile/g' EngineUtilMakefile && \
		$(SEDCOM) -i -e 's/\/usr\/include/$(OPENNI_INCLUDE)/g' EngineUtilMakefile && \
		cd -
	@-cd $(GIT_DIR)/Source/XnDeviceSensorV2 && \
		$(SEDCOM) -i -e 's/\/usr\/bin/$(ENGINE_BIN)/g' XnSensorClient.cpp && \
		cd -
	cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make && cd -
	@cp -r $(GIT_DIR)/Data/GlobalDefaults.ini lib/
	for file in `cat $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/install.sh | grep MODULES=.*$(LIBEXT).* | $(SEDCOM) 's/\"//g' | awk -F "=" {'print $$2'}`; do \
		$(LD_VAR)=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$$file lib; \
	done
	touch installed
#	@$(foreach file,\
#		$(shell echo $$(eval cat $(GIT_DIR)/Platform/$(PLATFORM)/Redist/install.sh | grep MODULES= | $(SEDCOM) 's/\"//g' | awk -F "=" {'print $$2'})),\
#		$(LD_VAR)=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$(file) lib;)
#	$(foreach file,$(wildcard lib/*.so),$(LD_VAR)=`rospack find openni`/lib rosrun openni niReg -r $(file) lib;)

clean:
	-cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make clean && cd -
	rm -rf $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/Final/*
	rm -f lib/*.so
	rm -rf bin config installed

wipe:
	rm -rf bin lib config installed
	rm -rf build patched rospack_nosubdirs
