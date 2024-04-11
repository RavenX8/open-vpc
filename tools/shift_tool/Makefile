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

build:
	cargo build $(release)

install:
	cp target/$(target)/$(prog) ~/bin/$(prog)$(extension)

clean:
	cargo clean


all: build install

help:
	@echo "usage: make $(prog) [debug=1]"
