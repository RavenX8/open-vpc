SUBDIRS := $(wildcard */.)

all clean: $(SUBDIRS) FORCE

$(SUBDIRS): FORCE
	$(MAKE) -C $@ $(MAKECMDGOALS)

FORCE:
