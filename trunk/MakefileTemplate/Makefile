#######################################################################
# low cost vision - generic make file
# version: v0.4.4
#######################################################################

#######################################################################
# config
#######################################################################

# type of project. may be 'binary' or 'library'
BUILDTYPE           := 

TARGET              := 

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
DEP_PROJ            := 

#######################################################################
# constants
#######################################################################

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
PROJ_PATHS = $(subst :, , $(LCV_PROJECT_PATH))
PROJ_DIRS := \
	$(foreach DEP, $(DEP_PROJ), \
		$(foreach PATH, $(PROJ_PATHS), \
			$(foreach DIR, $(dir $(wildcard $(patsubst %/,%,$(PATH))/*/)), \
				$(if $(subst $(notdir $(patsubst %/, %, $(DIR))),,$(DEP)),,\
					$(DIR) \
				) \
			) \
		) \
	)
PROJ_DIRS := $(strip $(PROJ_DIRS))
ifneq ($(words $(DEP_PROJ)), $(words $(PROJ_DIRS)))
$(error number of specified dependency projects does not match number of found dependency projects)
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
LFLAGS              += $(foreach LP, $(LINKERPATHS), -L$(LP)) $(LIBFLAGS)

# check if all packages exists according pkg-config
# extract flags from pkg-config
$(foreach PKG, $(PKGCONF_LIBRARIES), \
	$(if $(subst 0,,$(shell pkg-config --exists $(PKG); echo $$?)), \
		$(error library "$(PKG)" was not found by pkg-config), \
	) \
)
ifneq ($(words $(PKGCONF_LIBRARIES)), 0)
CXXFLAGS += $(shell pkg-config --cflags $(PKGCONF_LIBRARIES))
LFLAGS   += $(shell pkg-config --libs $(PKGCONF_LIBRARIES))
endif

# files removed during clean
CLEANFILES           = $(OBJS) $(DFILES) $(TARGET)

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

# recursively make all dependency projects
.PHONY: dep
dep:
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
	@$(PRINTF) '$(ANSI_BLUE)CXX\t$<$(ANSI_RESET)\n'
	$(COMPILE.cpp) $(OUTPUT_OPTION) $<

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
	@$(PRINTF) '$(ANSI_GREEN)LINK\t$^$(ANSI_RESET)\n'
	$(CXX) $(OBJS) -o $@ $(LFLAGS)

$(BINDIR):
	@$(PRINTF) '$(ANSI_YELLOW)MKDIR\t$@$(ANSI_RESET)\n'
	@-mkdir -p $(BINDIR)
else
ifeq ($(BUILDTYPE),library)
$(TARGET): $(OBJS) | $(LIBDIR)
	@$(PRINTF) '$(ANSI_MAGENTA)AR\t$@$(ANSI_RESET)\n'
	ar r $(ARFLAGS) $@ $^

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
