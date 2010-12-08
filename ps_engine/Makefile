all: installed

GIT_DIR = build/ps_engine
GIT_URL = https://github.com/ros-pkg-git/Sensor.git
#GIT_PATCH = 
GIT_REVISION=ros

ENGINE_BIN=$(shell echo $$(eval rospack find ps_engine | sed 's/\//\\\//g')\\/bin)
OPENNI_INCLUDE=$(shell echo $$(eval rospack find openni | sed 's/\//\\\//g')\\/include)
OPENNI_LIB=$(shell echo $$(eval rospack find openni | sed 's/\//\\\//g')\\/lib)

PLATFORM=Linux-x86
MODULES="libXnDeviceSensorV2.so libXnDeviceFile.so"

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	@-mkdir -p bin lib config install
	@-cd $(GIT_DIR)/Source/XnDeviceSensorV2 && \
		sed -i -e 's/\/usr\/bin/$(ENGINE_BIN)/g' XnSensorClient.cpp && \
		cd -
	@-cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && \
		sed -i -e 's/\/usr\/include\/ni\/CommonMakefile/$(OPENNI_INCLUDE)\/ni\/CommonMakefile/g' EngineLibMakefile && \
		sed -i -e 's/\/usr\/include/$(OPENNI_INCLUDE)/g' EngineLibMakefile && \
		sed -i -e 's/BIN_DIR/LIB_DIRS=$(OPENNI_LIB)\nBIN_DIR/g' EngineLibMakefile && \
		cd -
	@-cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Utils && \
		sed -i -e 's/\/usr\/include\/ni\/CommonMakefile/$(OPENNI_INCLUDE)\/ni\/CommonMakefile/g' EngineUtilMakefile && \
		sed -i -e 's/\/usr\/include/$(OPENNI_INCLUDE)/g' EngineUtilMakefile && \
		sed -i -e 's/BIN_DIR/LIB_DIRS=$(OPENNI_LIB)\nBIN_DIR/g' EngineUtilMakefile && \
		cd -
	cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make redist && cd -
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Redist/Bin/* bin/
	@cp -r $(GIT_DIR)/Data/GlobalDefaults.ini lib/
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Redist/Lib/* lib/
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Redist/Config/* config/
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Redist/Install/* install/
	@-cd bin && ln -s ../lib/* . && ln -s `rospack find openni`/lib/* . && cd -
	touch installed
	for file in `cat $(GIT_DIR)/Platform/$(PLATFORM)/Redist/install.sh | grep MODULES= | sed 's/\"//g' | awk -F "=" {'print $$2'}`; do \
		LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$$file lib; \
	done
#	@$(foreach file,\
#		$(shell echo $$(eval cat $(GIT_DIR)/Platform/$(PLATFORM)/Redist/install.sh | grep MODULES= | sed 's/\"//g' | awk -F "=" {'print $$2'})),\
#		LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$(file) lib;)
#	$(foreach file,$(wildcard lib/*.so),LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r $(file) lib;)

clean:
	-cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make clean && cd -
	rm -rf $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/Final/*
	rm -rf bin lib config install installed

wipe:
	rm -rf bin lib config install installed
	rm -rf build patched rospack_nosubdirs
