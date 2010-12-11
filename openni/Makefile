all: installed

GIT_DIR = build/openni
GIT_URL = https://github.com/OpenNI/OpenNI.git
#GIT_PATCH = redist.patch
GIT_REVISION=master

OPENNI=$(shell echo $$(eval rospack find openni | sed 's/\//\\\//g'))
OPENNI_LIB=$(shell echo $$(eval rospack find openni | sed 's/\//\\\//g')\\/lib)
PLATFORM=Linux-x86

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	@-mkdir -p bin include/ni lib
	@echo "### Patching the main Makefile for OpenNI..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && \
		sed -i -e 's/CFG=$$(CFG)//g' Makefile && \
		cd -
	@echo "### Patching the output library for OpenNI..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/OpenNI && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse3\nBIN_DIR=$(OPENNI)/g' Makefile && \
		cd -
	@echo "### Patching the output library for all library Modules..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Modules && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' nimCodecs/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' nimMockNodes/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' nimRecorder/Makefile && \
		cd -
	@echo "### Patching the output location for all samples..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Samples && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiViewer/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiAudioSample/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiBackRecorder/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiConvertXToONI/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiCRead/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiRecordSynthetic/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiSampleModule/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiSimpleCreate/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiSimpleRead/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiSimpleViewer/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiUserTracker/Makefile && \
		cd -
	@echo "### Patching the output location for all tools..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Utils && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' niLicense/Makefile && \
		sed -Ei -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse3\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' niReg/Makefile && \
		cd -
	@echo "### Patching the location of modules.xml..."
	@-cd $(GIT_DIR)/Source/OpenNI && \
		sed -Ei -e 's/\/var\/lib\/ni\/modules.xml/$(OPENNI_LIB)\/modules.xml/g' XnModuleLoader.cpp && \
		sed -Ei -e 's/\/var\/lib\/ni\/licenses.xml/$(OPENNI_LIB)\/licenses.xml/g' XnLicensing.cpp && \
		cd -
	@echo "### Patching the location of SamplesConfig.xml..."
	@-cd $(GIT_DIR)/Samples && \
		sed -Ei -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiAudioSample/NiAudioSample.cpp && \
		sed -Ei -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiViewer/NiViewer.cpp && \
		sed -Ei -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiRecordSynthetic/NiRecordSynthetic.cpp && \
		sed -Ei -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiSimpleViewer/NiSimpleViewer.cpp && \
		sed -Ei -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiSimpleRead/NiSimpleRead.cpp && \
		sed -Ei -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiUserTracker/main.cpp && \
		cd -
	cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make && cd -
	@cp -r $(GIT_DIR)/Include/* include/ni
	@cp $(GIT_DIR)/Data/SamplesConfig.xml lib/
	@cp $(GIT_DIR)/Platform/$(PLATFORM)/Build/CommonMakefile include/ni/
	@echo "### Register all modules..."
	for file in `cat $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/install.sh | grep MODULES= | sed 's/\"//g' | awk -F "=" {'print $$2'}`; do \
		LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$$file lib; \
	done
	touch installed
#	@patch -p0 < includes.patch

clean:
	-cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make clean && cd -
	rm -rf $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/Final/*
	rm -f lib/*.so
	rm -rf bin include installed

wipe:
	rm -rf bin include lib installed
	rm -rf build patched rospack_nosubdirs
