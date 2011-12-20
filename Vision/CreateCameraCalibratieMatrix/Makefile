#######################################################################
# low cost vision - configuration make file
# needs path to Makefile.generic in LCV_PROJECT_MAKEFILE
# version: v1.0.0
#######################################################################

#######################################################################
# config
#######################################################################

# type of project. may be 'binary' or 'library'
BUILDTYPE           := library

# name of target binary or library
TARGET              := CreateCameraCalibratieMatrix

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
PKGCONF_LIBRARIES   := opencv

# libraries that are linked against with '-l'
LIBRARIES           := boost_filesystem

# include paths that will be included using '-I'
EXTINCLUDEPATHS     := 

#linker paths that will be included using '-L'
LINKERPATHS         :=	opencv_core \
			opencv_calib3d \
			opencv_highgui \
			opencv_imgproc

# projects that this project depends on
# paths in environment variable LCV_PROJECT_PATH will be searched for projects
DEP_PROJ            := 

#######################################################################
# constants
#######################################################################
ifeq ($(LCV_PROJECT_MAKEFILE), )
$(error LCV_PROJECT_MAKEFILE is empty)
endif

include $(LCV_PROJECT_MAKEFILE)
