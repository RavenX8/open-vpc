prog :=shift_tool

debug ?=

$(info debug is $(debug))

ifdef debug
  release :=
  target :=debug
  extension :=-debug
else
  release :=--release
  target :=release
  extension :=
endif

ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

build:
	cargo build $(release)

install:
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 0755 target/$(target)/$(prog)$(extension) $(DESTDIR)$(PREFIX)/bin

clean:
	cargo clean


all: build install

help:
	@echo "usage: make $(prog) [debug=1]"
