#######################################################################
# low cost vision - generic make file
# version: v0.5.1/2
#######################################################################

#######################################################################
# config
#######################################################################

# type of project. may be 'binary' or 'library'
BUILDTYPE           := binary

# name of target binary or library
TARGET              := huniplacer_gui

# virtual path
VPATH               :=

# c++ compiler
CXX                 := g++

# c++ compiler flags
CXXFLAGS            := -Wall -g3

# preprocessor flags
CPPFLAGS            := $(shell wx-config --cxxflags)

# linker flags
LFLAGS              := $(shell wx-config --libs)

# arguments passed to 'ar' when archiving '.a' files
ARFLAGS             := 

# libraries that will be included by pkg-config
PKGCONF_LIBRARIES   :=

# libraries that are linked against with '-l'
LIBRARIES           := modbus boost_thread

# include paths that will be included using '-I'
EXTINCLUDEPATHS     := 

#linker paths that will be included using '-L'
LINKERPATHS         := 

# projects that this project depends on
# paths in environment variable LCV_PROJECT_PATH will be searched for projects
DEP_PROJ            := huniplacer

#######################################################################
# constants
#######################################################################

# misc settings
.SUFFIXES:
SHELL                = /bin/sh
PRINTF               = /usr/bin/printf
DEPGEN               = $(CXX) -M $(CXXFLAGS) $(CPPFLAGS) $(TARGET_ARCH)

# directories
BINDIR              := bin
LIBDIR              := lib
INCLUDEDIR          := include
SRCDIR              := src
BUILDDIR            := build

# ANSI color escapes
ifeq ($(TERM), xterm)     
ANSI_RED            := \e[1;31m
ANSI_YELLOW         := \e[1;33m
ANSI_GREEN          := \e[1;32m
ANSI_CYAN           := \e[1;36m
ANSI_BLUE           := \e[1;34m
ANSI_MAGENTA        := \e[1;35m
ANSI_RESET          := \e[0m
endif

# search dependency projects
# append to PROJ_DIRS
PROJ_PATHS := $(strip $(subst :, , $(LCV_PROJECT_PATH)))
DEP_PROJ   := $(strip $(DEP_PROJ))
ifeq ($(PROJ_PATHS), )
ifneq ($(DEP_PROJ), )
$(error dependency projects were specified but LCV_PROJECT_PATH is empty)
endif
endif
PROJ_DIRS  := \
	$(foreach DEP,$(DEP_PROJ), \
        $(eval DEP_FOUND := ) \
		$(foreach PATH, $(PROJ_PATHS), \
			$(foreach DIR, $(dir $(wildcard $(patsubst %/,%,$(PATH))/*/)), \
				$(if $(subst $(notdir $(patsubst %/, %, $(DIR))),,$(DEP)),, \
                    $(eval DEP_FOUND := found) \
					$(DIR) \
				) \
			) \
			$(if $(DEP_FOUND),,$(error can not find project $(DEP))) \
		) \
	)
PROJ_DIRS := $(strip $(PROJ_DIRS))

# TODO implement project dependecy crap generation
# generate and execute project expose script
#VOID := $(shell \
#    if [ ! -e $(BUILDDIR)/expose.sh ] ; then \
#        mkdir -p $(BUILDDIR); \
#        touch $(BUILDDIR)/expose.sh; \
#        $(MAKE) expose; \
#    fi)
#VOID := $(shell $(SHELL) $(BUILDDIR)/expose.sh)
#$(error wtf)

# make depenency projects expose their important settings
#$(foreach PRJ, $(PROJ_DIRS), $(shell $(MAKE) -C $(PRJ) expose))
#PKGCONF_LIBRARIES += $(LCV_EXP_PKGCONF_LIBRARIES)
#LIBRARIES         += $(LCV_EXP_LIBRARIES)
#EXTINCLUDEPATHS   += $(LCV_EXP_EXTINCLUDEPATHS)
#LINKERPATHS       += $(LCV_EXP_LINKERPATHS)

# check if all packages exists according pkg-config
$(foreach PKG, $(PKGCONF_LIBRARIES), \
	$(if $(subst 0,,$(shell pkg-config --exists $(PKG); echo $$?)), \
		$(error library $(PKG) was not found by pkg-config), \
	) \
)
ifneq ($(words $(PKGCONF_LIBRARIES)), 0)
CXXFLAGS := $(shell pkg-config --cflags $(PKGCONF_LIBRARIES))
LFLAGS   := $(shell pkg-config --libs $(PKGCONF_LIBRARIES))
endif

# add paths of required projects
EXTINCLUDEPATHS     := $(foreach PRJ, $(PROJ_DIRS), $(PRJ)$(INCLUDEDIR)) $(EXTINCLUDEPATHS)
LINKERPATHS         := $(foreach PRJ, $(PROJ_DIRS), $(PRJ)$(LIBDIR)) $(LINKERPATHS)
LIBRARIES           := \
	$(foreach PRJ, $(PROJ_DIRS), \
		$(foreach LIB, $(wildcard $(PRJ)$(LIBDIR)/lib*.a), \
			$(patsubst lib%.a,%,$(notdir $(LIB))) \
        ) \
	) $(LIBRARIES)

# append headers and sourcefiles to virtual path
VPATH               += $(foreach PTH, $(strip $(SRCDIR)) $(strip $(EXTINCLUDEPATHS)) $(strip $(INCLUDEDIR)),$(PTH):)

# list of sourcefiles
SOURCES             := $(foreach SRCFLDR, $(SRCDIR), $(wildcard $(SRCFLDR)/*.cpp))

# list of dependency files
DFILES              := $(foreach X, $(subst .cpp,.d,$(notdir $(SOURCES))), $(BUILDDIR)/$(X)) 

# list objects
OBJS                := $(foreach X, $(subst .cpp,.o,$(notdir $(SOURCES))), $(BUILDDIR)/$(X))

# list '-l' flags
LIBFLAGS            := $(foreach LIB, $(LIBRARIES), -l$(LIB))

# append external includes
CPPFLAGS            += $(foreach INC, $(EXTINCLUDEPATHS), -I$(INC))

# append local includes
CPPFLAGS            += $(foreach INC, $(INCLUDEDIR), -I$(INC))

# append '-L' paths (from LINKERPATHS) and LIBFLAGS to LFLAGS
LFLAGS              := $(foreach LP, $(LINKERPATHS), -L$(LP)) $(LIBFLAGS) $(LFLAGS)

# files removed during clean
CLEANFILES           = $(OBJS) $(DFILES) $(TARGET) $(BUILDDIR)/expose.sh

# directories removed during clean
CLEANDIRS            = $(BUILDDIR) $(BINDIR) $(LIBDIR)

# further specify target: binary or library
BUILDTYPE := $(strip $(BUILDTYPE))
ifeq ($(BUILDTYPE),binary)
TARGET := $(BINDIR)/$(TARGET)
else
ifeq ($(BUILDTYPE),library)
TARGET := $(LIBDIR)/lib$(TARGET).a
else
$(error unknown build type)
endif
endif

#######################################################################
# rules
#######################################################################

.PHONY: all
all: $(TARGET)

#.PHONY: expose
#expose:
#	@export LCV_EXP_PKGCONF_LIBRARIES=
#	@export LCV_EXP_LIBRARIES=
#	@export LCV_EXP_EXTINCLUDEPATHS=
#	@export LCV_EXP_LINKERPATHS=
#	@ERR=0; \
#	for EXP in $(PROJ_DIRS); do \
#		$(MAKE) -C "$$EXP" r_expose; \
#		if [ "$$?" -ne "0" ] ; then \
#			ERR=1; \
#			break; \
#		fi; \
#	done; \
#	if [ "$$ERR" -ne "0" ] ; then \
#		false; \
#	else \
#		true; \
#	fi
#	@export LCV_EXP_PKGCONF_LIBRARIES=$(strip $(LCV_EXP_PKGCONF_LIBRARIES)) $(PKGCONF_LIBRARIES)
#	@export LCV_EXP_LIBRARIES=$(strip $(LCV_EXP_LIBRARIES)) $(LCV_EXP_LIBRARIES)
#	@export LCV_EXP_EXTINCLUDEPATHS=$(strip $(LCV_EXP_EXTINCLUDEPATHS)) $(EXTINCLUDEPATHS)
#	@export LCV_EXP_LINKERPATHS=$(strip $(LCV_EXP_LINKERPATHS)) $(LINKERPATHS)
#	@echo 'export LCV_EXP_PKGCONF_LIBRARIES=$(LCV_EXP_PKGCONF_LIBRARIES)' > $(BUILDDIR)/expose.sh
#	@echo 'export LCV_EXP_LIBRARIES=$(LCV_EXP_LIBRARIES)' >> $(BUILDDIR)/expose.sh
#	@echo 'export LCV_EXP_EXTINCLUDEPATHS=$(LCV_EXP_EXTINCLUDEPATHS)' >> $(BUILDDIR)/expose.sh
#	@echo 'export LCV_EXP_LINKERPATHS=$(LCV_EXP_LINKERPATHS)' >> $(BUILDDIR)/expose.sh

#.PHONY: r_expose
#r_expose:
#	@ERR=0; \
#	for EXP in $(PROJ_DIRS); do \
#		$(MAKE) -C "$$EXP" r_expose; \
#		if [ "$$?" -ne "0" ] ; then \
#			ERR=1; \
#			break; \
#		fi; \
#	done; \
#	if [ "$$ERR" -ne "0" ] ; then \
#		false; \
#	else \
#		true; \
#	fi
#	@export LCV_EXP_PKGCONF_LIBRARIES=$(strip $(LCV_EXP_PKGCONF_LIBRARIES)) $(PKGCONF_LIBRARIES)
#	@export LCV_EXP_LIBRARIES=$(strip $(LCV_EXP_LIBRARIES)) $(LCV_EXP_LIBRARIES)
#	@export LCV_EXP_EXTINCLUDEPATHS=$(strip $(LCV_EXP_EXTINCLUDEPATHS)) $(EXTINCLUDEPATHS)
#	@export LCV_EXP_LINKERPATHS=$(strip $(LCV_EXP_LINKERPATHS)) $(LINKERPATHS)

# recursively make al dependency projects
.PHONY: dep
dep:
#	@if [ -e $(BUILDDIR)/expose.sh ] ; then \
#		$(PRINTF) '$(ANSI_RED)RM\t$(BUILDDIR)/expose.sh$(ANSI_RESET)\n'; \
#		rm $(BUILDDIR)/expose.sh; \
#	fi
	@ERR=0; \
	for DEP in $(PROJ_DIRS); do \
		$(MAKE) -C "$$DEP" dep; \
		if [ "$$?" -ne "0" ] ; then \
			ERR=1; \
			break; \
		fi; \
		$(MAKE) -C "$$DEP"; \
		if [ "$$?" -ne "0" ] ; then \
			ERR=1; \
			break; \
		fi; \
	done; \
	if [ "$$ERR" -ne "0" ] ; then \
		false; \
	else \
		true; \
	fi

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp | $(BUILDDIR)
	@$(PRINTF) '$(ANSI_BLUE)$(COMPILE.cpp) $(OUTPUT_OPTION) $<$(ANSI_RESET)\n'
	@$(COMPILE.cpp) $(OUTPUT_OPTION) $<

$(BUILDDIR):
	@$(PRINTF) '$(ANSI_YELLOW)MKDIR\t$@$(ANSI_RESET)\n'
	@-mkdir -p $(BUILDDIR)

-include $(DFILES)

$(BUILDDIR)/%.d: $(SRCDIR)/%.cpp | $(BUILDDIR)
	@$(PRINTF) '$(ANSI_CYAN)D\t$@$(ANSI_RESET)\n'
	@$(DEPGEN) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,$(BUILDDIR)\/\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

# create different rules for binary and library projects
ifeq ($(BUILDTYPE),binary)
$(TARGET): $(OBJS) | $(BINDIR)
	@$(PRINTF) '$(ANSI_GREEN)$(CXX) $(OBJS) -o $@ $(LFLAGS)$(ANSI_RESET)\n'
	@$(CXX) $(OBJS) -o $@ $(LFLAGS)

$(BINDIR):
	@$(PRINTF) '$(ANSI_YELLOW)MKDIR\t$@$(ANSI_RESET)\n'
	@-mkdir -p $(BINDIR)
else
ifeq ($(BUILDTYPE),library)
$(TARGET): $(OBJS) | $(LIBDIR)
	@$(PRINTF) '$(ANSI_MAGENTA)AR\t$@$(ANSI_RESET)\n'
	@ar r $(ARFLAGS) $@ $^

$(LIBDIR):
	@$(PRINTF) '$(ANSI_YELLOW)MKDIR\t$@$(ANSI_RESET)\n'
	@-mkdir -p $(LIBDIR)
endif
endif

.PHONY: clean
clean:
	@$(foreach FILE, $(strip $(CLEANFILES)),\
		if [ -e $(FILE) ] ; then \
			$(PRINTF) '$(ANSI_RED)RM\t$(FILE)$(ANSI_RESET)\n'; \
			rm $(FILE); \
		fi;\
	)
	@$(foreach DIR, $(strip $(CLEANDIRS)),\
		if [ -d $(DIR) ] ; then \
			$(PRINTF) '$(ANSI_RED)RMDIR\t$(DIR)$(ANSI_RESET)\n'; \
			rmdir -p $(DIR); \
		fi;\
	) true
