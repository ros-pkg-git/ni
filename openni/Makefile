all: installed

GIT_DIR = build/openni
GIT_URL = https://github.com/OpenNI/OpenNI.git
#GIT_PATCH = redist.patch
# unstable branch as of 2011-01-05
GIT_REVISION=cb4b7981e0232b2f772b220752ae49e1d211bf94

SEDCOM=sed
LD_VAR=LD_LIBRARY_PATH
LIBEXT=so
UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
  SEDCOM=gsed
  LD_VAR=DYLD_LIBRARY_PATH
  LIBEXT=dylib
endif

OPENNI=$(shell echo $$(eval rospack find openni | $(SEDCOM) 's/\//\\\//g'))
OPENNI_LIB=$(shell echo $$(eval rospack find openni | $(SEDCOM) 's/\//\\\//g')\\/lib)
PLATFORM=Linux-x86


include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	@-mkdir -p bin include/ni lib
	@echo "### Patching the main Makefile for OpenNI..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && \
		$(SEDCOM) -i -e 's/CFG=$$(CFG)//g' Makefile && \
		cd -
	@echo "### Patching the output library for OpenNI..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/OpenNI && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse2\nBIN_DIR=$(OPENNI)/g' Makefile && \
		cd -
	@echo "### Patching the output library for all library Modules..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Modules && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' nimCodecs/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' nimMockNodes/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=lib\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' nimRecorder/Makefile && \
		cd -
	@echo "### Patching the output location for all samples..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Samples && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiViewer/Makefile && \
		$(SEDCOM) -i -e 's/include/USED_LIBS+=GL\ninclude/g' NiViewer/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiAudioSample/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiBackRecorder/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiConvertXToONI/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiCRead/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiRecordSynthetic/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiSampleModule/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiSimpleCreate/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiSimpleRead/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiSimpleViewer/Makefile && \
		$(SEDCOM) -i -e 's/include/USED_LIBS+=GL\ninclude/g' NiSimpleViewer/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' NiUserTracker/Makefile && \
		$(SEDCOM) -i -e 's/include/USED_LIBS+=GL\ninclude/g' NiUserTracker/Makefile && \
		cd -
	@echo "### Patching the output location for all tools..."
	@cd $(GIT_DIR)/Platform/$(PLATFORM)/Build/Utils && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' niLicense/Makefile && \
		$(SEDCOM) -i -e 's/BIN_DIR = ..\/..\/..\/Bin/CFG=bin\nCFLAGS+=-O3 -msse2\nLDFLAGS+=-Wl,-rpath,$(OPENNI_LIB)\nLIB_DIRS=$(OPENNI_LIB)\nBIN_DIR=$(OPENNI)/g' niReg/Makefile && \
		cd -
	@echo "### Patching the location of modules.xml..."
	@-cd $(GIT_DIR)/Source/OpenNI && \
		$(SEDCOM) -i -e 's/\/var\/lib\/ni\/modules.xml/$(OPENNI_LIB)\/modules.xml/g' XnModuleLoader.cpp && \
		$(SEDCOM) -i -e 's/\/var\/lib\/ni\/licenses.xml/$(OPENNI_LIB)\/licenses.xml/g' XnLicensing.cpp && \
		cd -
	@echo "### Patching the location of SamplesConfig.xml..."
	@-cd $(GIT_DIR)/Samples && \
		$(SEDCOM) -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiAudioSample/NiAudioSample.cpp && \
		$(SEDCOM) -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiViewer/NiViewer.cpp && \
		$(SEDCOM) -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiRecordSynthetic/NiRecordSynthetic.cpp && \
		$(SEDCOM) -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiSimpleViewer/NiSimpleViewer.cpp && \
		$(SEDCOM) -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiSimpleRead/NiSimpleRead.cpp && \
		$(SEDCOM) -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiUserTracker/main.cpp && \
		cd -
	cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make core samples && cd -
	@cp -r $(GIT_DIR)/Include/* include/ni
	@cp $(GIT_DIR)/Data/SamplesConfig.xml lib/
	@cp $(GIT_DIR)/Platform/$(PLATFORM)/Build/CommonMakefile include/ni/
	@echo "### Register all modules..."
	for file in `cat $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/install.sh | grep MODULES=.*$(LIBEXT).* | $(SEDCOM) 's/\"//g' | awk -F "=" {'print $$2'}`; do \
		${LD_VAR}=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$$file lib; \
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
