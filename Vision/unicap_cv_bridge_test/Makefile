#######################################################################
# low cost vision - configuration make file
# needs Makefile.generic in LCV_PROJECT_PATH
# version: v1.0.0
#######################################################################

#######################################################################
# config
#######################################################################

# type of project. may be 'binary' or 'library'
BUILDTYPE           := binary

# name of target binary or library
TARGET              := unicap_cv_bridge_test

# virtual path
VPATH               :=

# c++ compiler
CXX                 := g++

# c++ compiler flags
CXXFLAGS            := -Wall -g3

# preprocessor flags
CPPFLAGS            := 

# linker flags
LFLAGS              := 

# arguments passed to 'ar' when archiving '.a' files
ARFLAGS             := 

# libraries that will be included by pkg-config
PKGCONF_LIBRARIES   := libunicap opencv

# libraries that are linked against with '-l'
LIBRARIES           := boost_thread

# include paths that will be included using '-I'
EXTINCLUDEPATHS     := 

#linker paths that will be included using '-L'
LINKERPATHS         := 

# projects that this project depends on
# paths in environment variable LCV_PROJECT_PATH will be searched for projects
DEP_PROJ            := unicap_cv_bridge

#######################################################################
# constants
#######################################################################
ifeq ($(LCV_PROJECT_MAKEFILE), )
$(error LCV_PROJECT_MAKEFILE is empty)
endif

include $(LCV_PROJECT_MAKEFILE)
