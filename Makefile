.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
AR      = ar
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
######################################
# Project Name (generate executable with this name)
TARGET = cs296_29_exe
FOLDER = generate

# Boolean for deciding library linking mode
SHARED_LIB = FALSE                                    # FALSE for static linking

# Project Paths
PROJECT_ROOT=./
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
EXT_SRC=$(EXTERNAL_ROOT)/src
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/myobjs
BINDIR = $(PROJECT_ROOT)/mybins
DOCDIR = $(PROJECT_ROOT)/doc

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-p -L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJ_FILES := $(wildcard $(OBJDIR)/*[!main].o)
DATA_FILES := $(wildcard $(PROJECT_ROOT)/data/*[!pnp_austen].*)

.PHONY: all setup exe doc report plot clean distclean dist install

all:    setup exe doc report

###################################################################################################################

install:
	@mkdir -p $(FOLDER); \
	@cp $(BINDIR)/$(TARGET) $(FOLDER); \
	@cp -R $(DOCDIR)/html $(FOLDER); \
	@cp $(DOCDIR)/cs296_report_29.pdf;

setup:
	@$(ECHO) -n "Setting up directories myobjs, mybins, mylibs..."
	@mkdir -p myobjs
	@mkdir -p mybins
	@$(ECHO) "Done"
	@if test -d $(EXTERNAL_ROOT)/include/Box2D -a -d $(EXTERNAL_ROOT)/lib/Box2D; \
	then $(ECHO) "Box2D package found already installed";\
	else $(ECHO) "Box2D package not found...Installing now..."; \
	$(ECHO) "Untarring Box2d.tgz..."; \
	tar -xvzf $(EXT_SRC)/Box2D.tgz -C $(EXT_SRC)/; \
	$(ECHO) "Done"; \
	$(ECHO) "Creating directory named build296..."; \
	mkdir $(EXT_SRC)/Box2D/build296; \
	$(ECHO) "Done"; \
	$(ECHO) "--------------------------------------------------------------------------------------"; \
	$(ECHO) "Creating Makefile for Box2D using cmake..."; \
	cd $(EXT_SRC)/Box2D/build296/; pwd; cmake ../; \
	$(ECHO) "Done"; \
	$(ECHO) ""; \
	$(ECHO) "Installing Box2D using make...";\
	cd $(EXT_SRC)/Box2D/build296; make; make install; \
	$(ECHO) "Done";\
	fi;

-include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) -fPIC $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
        then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
        elif test -s temp.log; \
        then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
        else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
        fi;
	@$(RM) -f temp.log temp.err

exe: $(OBJS)   
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -o $(PROJECT_ROOT)/mybins/$(TARGET) $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err


###################################################################################################################

doc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"

report:
	@cd  $(PROJECT_ROOT)/doc \
	&& pdflatex  g29_prof_report.tex \

plot: 
	@cd ./scripts; \
	chmod +x *;\ 
	python3.3 g29_gen_csv.py;\ 
	echo -n "Finished script 1.\n";\ 
	python3.3 g29_gen_plots.py;\ 
	echo -n "Finished script 2.\n";\ 
	@$(ECHO) "Done"

###################################################################################################################

clean:
	@$(ECHO) -n "Cleaning up executables, object files..."
	@$(RM) -rf $(OBJDIR)/* $(BINDIR)/* $(DEPS) $(SRCDIR)/*~
	@$(RM) -rf $(DOCDIR)/*.aux $(DOCDIR)/*.bbl $(DOCDIR)/*.blg $(DOCDIR)/*.dvi $(DOCDIR)/*.log $(DOCDIR)/*.pdf $(DOCDIR)/html
	@$(ECHO) "Done"
	@rm -rf g29_release_prof.dat
	@rm -rf g29_debug_prof.dat
	@rm -rf release.png
	@rm -rf debug.png
	@rm -rf gmon.out
	@$(ECHO) -n "Cleaning up data and plots ..."
	@$(RM) -rf $(DATA_FILES) $(PROJECT_ROOT)/plots $(PROJECT_ROOT)/scripts/fit.log
	@$(ECHO) "Done"


distclean: clean
	@$(ECHO) -n "Uninstalling Box2D..."
	@$(RM) -rf $(EXTERNAL_ROOT)/include/* $(EXTERNAL_ROOT)/lib/* $(EXT_SRC)/Box2D $(DOCDIR)/html
	@$(RM) -rf $(BINDIR) $(OBJDIR)
	@$(ECHO) "Removed"

dist: distclean
	@cd ../; \
	tar cf cs296_g29_project.tar README.txt cs296_base_code; \
	gzip cs296_g29_project.tar;
