.SUFFIXES: .o .cpp

SDKVERSION=4.5.1
#At least DEBUG or OPTIMIZE has to be switched on
OPTIMIZE=off
DEBUG=on
WITHJPEGSUPPORT=off

CXX=g++
CXXFLAGS = -Wall -ansi -pedantic -Wextra -Winit-self -Wno-long-long -Wformat -Wenum-compare -Wconversion -Wsign-conversion -Wshadow
# -Wc++11-compat
INCLUDEPATH = -I/usr/include/boost/ -I$(CURDIR)/src
CXXFLAGS += $(INCLUDEPATH)
CPPFLAGS += $(INCLUDEPATH)
LIBS= -lrt -L/usr/lib -lboost_system -lboost_thread -lboost_date_time -lboost_program_options -lpthread
ADDITIONALUNITTESTLIBS=-lboost_unit_test_framework


ifeq ($(WITHJPEGSUPPORT),on)
	LIBS +=  -L /usr/lib/x86_64-linux-gnu -ljpeg
endif



IBEOSDK_LIBNAME=ibeosdk
export BASEDIR:=$(CURDIR)
IBEOSDK_LIBDIR=$(BASEDIR)/libs
export IBEOSDK_LIBDIR
export ADDITIONALUNITTESTLIBS

#determine the compiler flags depending on optimize/debug
ifeq ($(OPTIMIZE),on)
	CXXFLAGS+= -O3 -DNDEBUG
endif

ifeq ($(DEBUG),on)
	CXXFLAGS+= -g3
endif

#determine the output path and the library suffix depending on optimize/debug
ifeq ($(OPTIMIZE),on)
	ifeq ($(DEBUG), on)
		export BUILDTYPE:=optwithdbg
		export LIBSUFFIX:=od
	else
		export BUILDTYPE:=optimized
		export LIBSUFFIX:=
	endif
else
	ifneq ($(DEBUG), on)
		#$(error "At least DEBUG or OPTIMIZE has to be switched on")
	endif
	export BUILDTYPE:=debug
	export LIBSUFFIX:=d
endif

OBJFOLDER=$(CURDIR)/tmp/#
OBJFOLDER:=$(OBJFOLDER)/$(BUILDTYPE)
DEMOBINFOLDER=$(CURDIR)/bin/$(BUILDTYPE)
TOOLBINFOLDER=$(CURDIR)/bin/tools/$(BUILDTYPE)
UNITTESTBINFOLDER=$(CURDIR)/bin/unittests/$(BUILDTYPE)



TOOLSSOURCES=$(shell find src/tools -name "*.hpp" -o -name "*.cpp")
SKIPFILES=$(TOOLSSOURCES)
SOURCES=$(filter-out $(SKIPFILES), $(shell find . -name "*.cpp" -a ! -wholename "*unittests*"  -a ! -wholename "*demo*"  -a ! -wholename "*tool*" | sed 's_\./__'))
OBJECTS=$(SOURCES:%.cpp=%.o)

export DEMOBINFOLDER TOOLBINFOLDER UNITTESTBINFOLDER
export CXX CXXFLAGS CPPFLAGS INCLUDEPATH LIBS OBJFOLDER  SKIPFILES
export IBEOSDK_LIBNAME SDKVERSION
export WITHJPEGSUPPORT

DEMOSOURCES=$(shell find . -name "*.cpp" -a -wholename "*demo*")
DEMOBINARIES=$(DEMOSOURCES:%.cpp=%)

TESTSOURCES=$(shell find . -name "*.cpp" -a -wholename "*unittest*")
TESTBINARIES=$(TESTSOURCES:%.cpp=%)



################################################################################
#  Library Targets
################################################################################

.PHONY : all static shared

all shared:  | libsdir 
	$(MAKE) -f Makefile2 LIBTYPE=shared

static: | libsdir 
	$(MAKE) -f Makefile2 LIBTYPE=static


################################################################################
#  Demo Targets
################################################################################
.PHONY : demos shareddemos staticdemos

shareddemos :  shared | bindir
	$(MAKE) -f Makefile2 shareddemos LIBTYPE=shared

# alias for shareddemos
demos : shared | bindir
	$(MAKE) -f Makefile2 shareddemos LIBTYPE=shared

staticdemos : static | bindir
	$(MAKE) -f Makefile2 staticdemos LIBTYPE=static


################################################################################
#  Tool Targets
################################################################################
.PHONY : tools sharedtools statictools

sharedtools :  shared | tooldir
	$(MAKE) -f Makefile2 sharedtools LIBTYPE=shared

# alias for sharedtools
tools : shared | tooldir
	$(MAKE) -f Makefile2 sharedtools LIBTYPE=shared

statictools : static | tooldir
	$(MAKE) -f Makefile2 statictools LIBTYPE=static


################################################################################
#  Test Targets
################################################################################
.PHONY : tests sharedtests statictests

sharedtests :  shared | unittestdir
	$(MAKE) -f Makefile2 sharedtests LIBTYPE=shared

# alias for sharedtests
tests : shared | unittestdir
	$(MAKE) -f Makefile2 sharedtests LIBTYPE=shared

statictests : static | unittestdir
	$(MAKE) -f Makefile2 statictests LIBTYPE=static




libsdir:
	@if [ ! -d libs ]; then mkdir libs; fi
bindir:
	@if [ ! -d $(DEMOBINFOLDER) ]; then mkdir -p $(DEMOBINFOLDER); fi

unittestdir: | bindir
	@if [ ! -d $(UNITTESTBINFOLDER) ]; then mkdir -p $(UNITTESTBINFOLDER); fi
	
tooldir: | bindir
	@if [ ! -d $(TOOLBINFOLDER) ]; then mkdir -p $(TOOLBINFOLDER); fi



################################################################################
#  Cleanup Targets
################################################################################

clean: | democlean
	@echo "Clean all compiled files for LIBTYPE=static and shared, DEBUG=$(DEBUG) and OPTIMIZE=$(OPTIMIZE)"
	@$(MAKE) -f Makefile2 clean LIBTYPE=shared
	@$(MAKE) -f Makefile2 clean LIBTYPE=static

clean_shared: | democlean_shared
	@echo "Clean all compiled files for LIBTYPE=shared, DEBUG=$(DEBUG) and OPTIMIZE=$(OPTIMIZE)"
	@$(MAKE) -f Makefile2 clean LIBTYPE=shared


clean_static: | democlean_static
	@echo "Clean all compiled files for LIBTYPE=static, DEBUG=$(DEBUG) and OPTIMIZE=$(OPTIMIZE)"
	@$(MAKE) -f Makefile2 clean LIBTYPE=static



democlean:
	@$(MAKE) -f Makefile2 democlean LIBTYPE=shared
	@$(MAKE) -f Makefile2 democlean LIBTYPE=static

democlean_shared:
	@$(MAKE) -f Makefile2 democlean LIBTYPE=shared

democlean_static:
	@$(MAKE) -f Makefile2 democlean LIBTYPE=static


toolclean:
	@$(MAKE) -f Makefile2 toolclean LIBTYPE=shared
	@$(MAKE) -f Makefile2 toolclean LIBTYPE=static

toolclean_shared:
	@$(MAKE) -f Makefile2 toolclean LIBTYPE=shared

toolclean_static:
	@$(MAKE) -f Makefile2 toolclean LIBTYPE=static



unittestclean:
	@$(MAKE) -f Makefile2 unittestclean LIBTYPE=shared
	@$(MAKE) -f Makefile2 unittestclean LIBTYPE=static

unittestclean_shared:
	@$(MAKE) -f Makefile2 unittestclean LIBTYPE=shared

unittestclean_static:
	@$(MAKE) -f Makefile2 unittestclean LIBTYPE=static






#	rm -f  $(DEMOBINARIES) $(DEMOBINARIES:%=%_shared)  $(DEMOBINARIES:%=%_static) *.d *.o

distclean:
	rm -rf libs
	rm -rf tmp
	rm -rf bin
	@$(MAKE) docsclean

docs:
	$(MAKE) -C src/ docs


# Target for internal use to prepare a release zip file
archive:
	$(MAKE) docs
	rm -f IbeoSDK$(SDKVERSION).zip
	git archive --format=zip -o IbeoSDK$(SDKVERSION).zip HEAD
	zip -9 IbeoSDK$(SDKVERSION).zip src/docs/html/*

docsclean:
	$(MAKE) -C src/ docsclean

