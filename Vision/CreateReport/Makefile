#######################################################################
# low cost vision - configuration make file
# needs path to Makefile.generic in LCV_PROJECT_MAKEFILE
# version: v1.0.0
#######################################################################

#######################################################################
# config
#######################################################################

# type of project. may be 'binary' or 'library'
BUILDTYPE           := binary
#BUILDTYPE           := library

TARGET              := CreateReportAlpha1

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
PKGCONF_LIBRARIES   :=

# libraries that are linked against with '-l'
LIBRARIES           := 

# include paths that will be included using '-I'
EXTINCLUDEPATHS     := 

#linker paths that will be included using '-L'
LINKERPATHS         := 

# projects that this project depends on
# paths in environment variable LCV_PROJECT_PATH will be searched for projects
DEP_PROJ            := ImageMetaDataWrapper

#######################################################################
# constants
#######################################################################
ifeq ($(LCV_PROJECT_MAKEFILE), )
$(error LCV_PROJECT_MAKEFILE is empty)
endif

include $(LCV_PROJECT_MAKEFILE)
