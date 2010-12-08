all: installed

GIT_DIR = build/openni
GIT_URL = https://ros-pkg-git@github.com/ros-pkg-git/OpenNI.git
#GIT_PATCH = redist.patch
GIT_REVISION=ros

OPENNI_LIB=$(shell echo $$(eval rospack find openni | sed 's/\//\\\//g')\\/lib)
PLATFORM=Linux-x86

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	@-mkdir -p bin include/ni lib
	@-cd $(GIT_DIR)/Source/OpenNI && \
		sed -i -e 's/\/var\/lib\/ni\/modules.xml/$(OPENNI_LIB)\/modules.xml/g' XnModuleLoader.cpp && \
		cd -
	@-cd $(GIT_DIR)/Samples && \
		sed -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiAudioSample/NiAudioSample.cpp && \
		sed -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiViewer/NiViewer.cpp && \
		sed -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiRecordSynthetic/NiRecordSynthetic.cpp && \
		sed -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiSimpleViewer/NiSimpleViewer.cpp && \
		sed -i -e 's/..\/..\/..\/..\/Data\/SamplesConfig.xml/$(OPENNI_LIB)\/SamplesConfig.xml/g' NiSimpleRead/NiSimpleRead.cpp && \
		cd -
	cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make redist && cd -
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Redist/Bin/* bin/
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Bin/Release/Sample* bin/
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Bin/Release/NiViewer bin/
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Redist/Lib/* lib/
	@-cd bin && ln -s ../lib/* . && cd -
	@cp -r $(GIT_DIR)/Platform/$(PLATFORM)/Redist/Include/* include/ni
	@cp $(GIT_DIR)/Data/SamplesConfig.xml lib/
	for file in `cat $(GIT_DIR)/Platform/$(PLATFORM)/Redist/install.sh | grep MODULES= | sed 's/\"//g' | awk -F "=" {'print $$2'}`; do \
		LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$$file lib; \
	done
#	@patch -p0 < includes.patch
	touch installed
#	$(foreach file,\
#		$(shell echo $$(eval cat $(GIT_DIR)/Platform/$(PLATFORM)/Redist/install.sh | grep MODULES= | sed 's/\"//g' | awk -F "=" {'print $$2'})),\
#		LD_LIBRARY_PATH=`rospack find openni`/lib rosrun openni niReg -r `pwd`/lib/$(file) lib;)
#	$(foreach file,$(wildcard lib/*.so),LD_LIBRARY_PATH=lib bin/niReg -r $(file) lib;)

clean:
	-cd $(GIT_DIR)/Platform/$(PLATFORM)/Build && make clean && cd -
	rm -rf $(GIT_DIR)/Platform/$(PLATFORM)/CreateRedist/Final/*
	rm -rf bin include lib installed

wipe:
	rm -rf bin include lib installed
	rm -rf build patched rospack_nosubdirs
